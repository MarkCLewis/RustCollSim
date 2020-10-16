use std::cell::RefCell;
use std::rc::{Rc, Weak};
use std::fmt::Display;

/*
    Code is derivative, I wrote most of a DLL then found a much more efficient algorithm here:
    https://www.reddit.com/r/programming/comments/7zbi6s/why_writing_a_doubly_linked_list_in_safe_rust_is/dunjytf/?context=0
*/

struct Node<T> {
    pub data: T,
    pub prev: Option<Weak<RefCell<Node<T>>>>,
    pub next: Option<Rc<RefCell<Node<T>>>>,
}

impl<T> Node<T> {

    pub fn new(data: T) -> Self {
        Self { data, prev: None, next: None }
    }

    pub fn append(node: &mut Rc<RefCell<Node<T>>>, data: T) -> Option<Rc<RefCell<Node<T>>>> {
        let is_last = node.borrow().next.is_none();
        if is_last { 
            let mut new_node = Node::new(data);
            new_node.prev = Some(Rc::downgrade(&node));
            let rc = Rc::new(RefCell::new(new_node));
            node.borrow_mut().next = Some(rc.clone());
            Some(rc)
        } else {
            if let Some(ref mut next) = node.borrow_mut().next {
                Self::append(next, data)
            } else { None }
        }
    }
}

struct List<T> {
    first: Option<Rc<RefCell<Node<T>>>>,
    last: Option<Rc<RefCell<Node<T>>>>,
}

impl<T> List<T> {
    pub fn new() -> Self {
        Self { first: None, last: None }
    }
    pub fn append(&mut self, data: T) {
        if let Some(ref mut next) = self.first {
            self.last = Node::append(next, data);
        } else {
            let f = Rc::new(RefCell::new(Node::new(data)));
            self.first = Some(f.clone());
            self.last = Some(f);
        }
    }
}

fn main() {
    let mut list = List::new();
    println!("{}", list);
    for i in 0..5 {
        list.append(i);
    }
    //goal is monstrosity, page 548 in CS2 textbook
    println!("Hello, doubly linked list!");
}