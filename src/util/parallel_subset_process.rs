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