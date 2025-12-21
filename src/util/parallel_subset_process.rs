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

pub fn parallel_subset_process_recur_mut1<A, F>(mut_slice: &mut [A], elements: &[usize], func: &F)
where
    A: Send + Sync,
    F: Fn(&mut A) + Send + Sync,
{
    fn recurse<A, F>(mut_slice: &mut [A], offset: usize, elements: &[usize], func: &F) 
    where
        A: Send + Sync,
        F: Fn(&mut A) + Send + Sync
    {
        if elements.len() == 1 {
            func(&mut mut_slice[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let mid_elem = elements[mid];
            let (left_mut_slice, right_mut_slice) = mut_slice.split_at_mut(mid_elem - offset);
            if elements.len() < 10 {
                recurse(left_mut_slice, offset, left, func);
                recurse(right_mut_slice, mid_elem,right, func);
            } else {
                join(
                    || recurse(left_mut_slice, offset, left, func),
                    || recurse(right_mut_slice, mid_elem, right, func),
                );
            }
        }
    }
    recurse(mut_slice, 0, elements, func);
}


pub fn parallel_subset_process_recur_mut<A, B, F>(slice: &[A], mut_slice: &mut [B], elements: &[usize], func: &F)
where
    A: Send + Sync,
    B: Send + Sync,
    F: Fn(&A, &mut B) + Send + Sync,
{
    fn recurse<A, B, F>(slice: &[A], mut_slice: &mut [B], offset: usize, elements: &[usize], func: &F) 
    where
        A: Send + Sync,
        B: Send + Sync,
        F: Fn(&A, &mut B) + Send + Sync
    {
        if elements.len() == 1 {
            func(&slice[elements[0]], &mut mut_slice[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let mid_elem = elements[mid];
            let (left_mut_slice, right_mut_slice) = mut_slice.split_at_mut(mid_elem - offset);
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

pub fn parallel_subset_process_recur_mut_res<A, B, C, F>(slice: &[A], mut_slice: &mut [B], elements: &[usize], results: &mut [C], func: &F)
where
    A: Send + Sync,
    B: Send + Sync,
    C: Send + Sync,
    F: Fn(usize, &A, &mut B) -> C + Send + Sync,
{
    fn recurse<A, B, C, F>(slice: &[A], mut_slice: &mut [B], offset: usize, elements: &[usize], results: &mut [C], func: &F) 
    where
        A: Send + Sync,
        B: Send + Sync,
        C: Send + Sync,
        F: Fn(usize, &A, &mut B) -> C + Send + Sync
    {
        if elements.len() == 1 {
            func(elements[0], &slice[elements[0]], &mut mut_slice[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let (left_results, right_results) = results.split_at_mut(mid);
            let mid_elem = elements[mid];
            let (left_mut_slice, right_mut_slice) = mut_slice.split_at_mut(mid_elem - offset);
            if elements.len() < 10 {
                recurse(slice, left_mut_slice, offset, left, left_results, func);
                recurse(slice, right_mut_slice, mid_elem,right, right_results, func);
            } else {
                join(
                    || recurse(slice, left_mut_slice, offset, left, left_results, func),
                    || recurse(slice, right_mut_slice, mid_elem, right, right_results, func),
                );
            }
        }
    }
    recurse(slice, mut_slice, 0, elements, results, func);
}

pub fn parallel_subset_process_recur_mut2<A, B, F>(mut_slice1: &mut [A], mut_slice2: &mut [B], elements: &[usize], func: &F)
where
    A: Default + Send + Sync,
    B: Default + Send + Sync,
    F: Fn(usize, &mut A, &mut B) + Send + Sync,
{
    fn recurse<A, B, F>(mut_slice1: &mut [A], mut_slice2: &mut [B], offset: usize, elements: &[usize], func: &F) 
    where
        A: Send + Sync,
        B: Default + Send + Sync,
        F: Fn(usize, &mut A, &mut B) + Send + Sync
    {
        if elements.len() == 1 {
            func(elements[0], &mut mut_slice1[elements[0] - offset], &mut mut_slice2[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let mid_elem = elements[mid];
            let (left_mut_slice1, right_mut_slice1) = mut_slice1.split_at_mut(mid_elem - offset);
            let (left_mut_slice2, right_mut_slice2) = mut_slice2.split_at_mut(mid_elem - offset);
            if elements.len() < 10 {
                recurse(left_mut_slice1, left_mut_slice2, offset, left, func);
                recurse(right_mut_slice1, right_mut_slice2, mid_elem,right, func);
            } else {
                join(
                    || recurse(left_mut_slice1, left_mut_slice2, offset, left, func),
                    || recurse(right_mut_slice1, right_mut_slice2, mid_elem, right, func),
                );
            }
        }
    }
    recurse(mut_slice1, mut_slice2, 0, elements, func);
}

pub fn parallel_subset_process_recur_mut2_res<A, B, C, F>(mut_slice1: &mut [A], mut_slice2: &mut [B], elements: &[usize], results: &mut [C], func: &F)
where
    A: Default + Send + Sync,
    B: Default + Send + Sync,
    C: Send + Sync,
    F: Fn(usize, &mut A, &mut B) -> C + Send + Sync,
{
    fn recurse<A, B, C, F>(mut_slice1: &mut [A], mut_slice2: &mut [B], offset: usize, elements: &[usize], results: &mut [C], func: &F) 
    where
        A: Send + Sync,
        B: Default + Send + Sync,
        C: Send + Sync,
        F: Fn(usize, &mut A, &mut B) -> C + Send + Sync
    {
        if elements.len() == 1 {
            results[0] = func(elements[0], &mut mut_slice1[elements[0] - offset], &mut mut_slice2[elements[0] - offset]);
        } else {
            let mid = elements.len() / 2;
            let (left, right) = elements.split_at(mid);
            let (results_left, results_right) = results.split_at_mut(mid);
            let mid_elem = elements[mid];
            let (left_mut_slice1, right_mut_slice1) = mut_slice1.split_at_mut(mid_elem - offset);
            let (left_mut_slice2, right_mut_slice2) = mut_slice2.split_at_mut(mid_elem - offset);
            if elements.len() < 10 {
                recurse(left_mut_slice1, left_mut_slice2, offset, left, results_left, func);
                recurse(right_mut_slice1, right_mut_slice2, mid_elem,right, results_right, func);
            } else {
                join(
                    || recurse(left_mut_slice1, left_mut_slice2, offset, left, results_left, func),
                    || recurse(right_mut_slice1, right_mut_slice2, mid_elem, right, results_right, func),
                );
            }
        }
    }
    recurse(mut_slice1, mut_slice2, 0, elements, results, func);
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