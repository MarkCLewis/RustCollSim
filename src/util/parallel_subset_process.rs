// Approaches to processing the elements of a slice in parallel where we only need to process a subset of the values

use rayon::{join};

// One approach is to recursively break the slice up and spawn threads then join on the
// two sides. This function assumes elements is sorted.
pub fn parallel_subset_process_recur<A, B, F>(slice: &[A], elements: &[usize], func: &F) -> Vec<B>
where
    A: Send + Sync,
    B: Default + Send + Sync,
    F: Fn(&A) -> B + Send + Sync,
{
    fn recurse<A, B, F>(slice: &[A], elements: &[usize], func: &F, result: &mut [B]) 
    where
        A: Send + Sync,
        B: Default + Send + Sync,
        F: Fn(&A) -> B + Send + Sync
    {
        if elements.len() == 1 {
            result[0] = func(&slice[elements[0]]);
        } else {
            let (left, right) = elements.split_at(elements.len() / 2);
            let (mut left_result, mut right_result) = result.split_at_mut(elements.len() / 2);
            if elements.len() < 10 {
                recurse(slice, left, func, &mut left_result);
                recurse(slice, right, func, &mut right_result);
            } else {
                join(
                    || recurse(slice, left, func, &mut left_result),
                    || recurse(slice, right, func, &mut right_result),
                );
            }
        }
    }
    let mut ret = Vec::new();
    ret.resize_with(elements.len(), Default::default);
    recurse(slice, elements, func, &mut ret);
    ret
}

// We need a version that modifies the slice.
pub fn parallel_subset_process_recur_mut<A, B, F>(slice: &[A], mut_slice: &mut [B], elements: &[usize], func: &F)
where
    A: Send + Sync,
    B: Default + Send + Sync,
    F: Fn(&A, &mut B) + Send + Sync,
{
    fn recurse<A, B, F>(slice: &[A], mut_slice: &mut [B], offset: usize, elements: &[usize], func: &F) 
    where
        A: Send + Sync,
        B: Default + Send + Sync,
        F: Fn(&A, &mut B) + Send + Sync
    {
        if elements.len() == 1 {
            func(&slice[elements[0]], &mut mut_slice[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let mid_elem = elements[mid];
            let (mut left_mut_slice, mut right_mut_slice) = mut_slice.split_at_mut(mid_elem - offset);
            if elements.len() < 10 {
                recurse(slice, left_mut_slice, offset, left, func);
                recurse(slice, right_mut_slice, mid_elem,right, func);
            } else {
                join(
                    || recurse(slice, left_mut_slice, offset, left, func),
                    || recurse(slice, right_mut_slice, mid_elem, right, func),
                );
            }
        }
    }
    recurse(slice, mut_slice, 0, elements, func);
}

// 0 1 | 2  | 3  4  | 5  6  7  8  9  | 10 11 12 13 14 15 16 17 18 19
// 0 5 | 10 | 15 20 | 25 30 35 40 45 | 50 55 60 65 70 75 80 85 90 95

// Another approach is to make a vector of slices that each begin with the one of the elements.
pub fn vector_of_independent_slices<'a, E>(slice: &'a [E], elements: &[usize]) -> Vec<&'a [E]> {
    let mut result: Vec<&'a [E]> = Vec::new();
    let mut slice = slice;
    for &element_index in elements.iter().rev() {
        let (first, rest) = slice.split_at(element_index);
        result.push(rest);
        slice = first;
    }
    result.reverse();
    result
}