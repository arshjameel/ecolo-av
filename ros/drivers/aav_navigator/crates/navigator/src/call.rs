//! One FollowPath call, from asking to the result, with no ROS (TODO 240).
//!
//! The follower is alive while it gives feedback (`follow_timeout`). A goal's feedback
//! must count from the moment the goal is asked for, not only once the follower's
//! answer to it ("accepted") has come: on 18 Nov, in CI's nightly sim, the navigator
//! aborted a goal with "no word from the follower in 4.0 s" while `mppi_lag` was driving
//! the car at 1 m/s. The silence began at the 10 s path renewal (`keep_path_age`), when
//! the follower dropped the old goal's feedback for the new one's, and the new goal's
//! feedback was read only after its answer.
//!
//! ```text
//! ask ──┬─ the answer: accepted ──────────────────▶ wait for its result
//!       │              refused ───────────────────▶ Refused
//!       └─ its feedback first (it must have taken the goal)
//!                       ──▶ ask for the result by the goal's id; the answer, if it
//!                           still comes, is only noted (`answered`)
//! every feedback, before or after the answer ─────▶ heard()
//! ```

use std::future::Future;
use std::pin::Pin;

use futures::future::{self, Either};
use futures::{FutureExt, Stream, StreamExt};

/// How a call to the follower ended.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Called<R> {
    /// The follower's answer refused the goal.
    Refused,
    /// The follower's result.
    Done {
        /// What the follower sent at the end.
        result: R,
        /// Its answer to the goal came. `false`: never, and only the goal's feedback
        /// showed it was taken (the result was asked for by the goal's id).
        answered: bool,
    },
}

/// What happened next, once the goal is taken.
enum Event<R> {
    Done(R),
    Heard,
    FeedbackEnded,
    Answered,
}

/// What came first after asking.
enum First<A> {
    Answer(A),
    Feedback(bool),
}

/// Follow one goal to its end.
///
/// `answer`: the follower's answer to the goal: `Some(the result)` if accepted, `None` if
/// refused. `feedback`: the goal's feedback, listened to from the moment it was asked for.
/// `by_id`: the result, asked for by the goal's id (when its feedback came before its
/// answer). `heard`: called at each feedback (the navigator's `follower_heard`).
pub async fn follow_call<R, Answer, ResultFut, Feedback, ById, ByIdFut>(
    answer: Answer,
    feedback: Feedback,
    by_id: ById,
    mut heard: impl FnMut(),
) -> Called<R>
where
    Answer: Future<Output = Option<ResultFut>>,
    ResultFut: Future<Output = R>,
    Feedback: Stream<Item = ()>,
    ById: FnOnce() -> ByIdFut,
    ByIdFut: Future<Output = R>,
{
    let mut answer = Box::pin(answer);
    let mut feedback = Box::pin(feedback);
    let first = match future::select(answer.as_mut(), feedback.next()).await {
        Either::Left((said, _)) => First::Answer(said),
        Either::Right((item, _)) => First::Feedback(item.is_some()),
    };
    // The result to wait for, and the answer if it is still to come.
    let (result, mut waiting) = match first {
        First::Answer(Some(result)) => (Either::Left(result), None),
        First::Answer(None) => return Called::Refused,
        First::Feedback(true) => {
            heard();
            (Either::Right(by_id()), Some(answer))
        }
        // No feedback at all: the answer alone.
        First::Feedback(false) => match answer.await {
            Some(result) => {
                return Called::Done {
                    result: result.await,
                    answered: true,
                }
            }
            None => return Called::Refused,
        },
    };
    let mut result = Box::pin(result);
    loop {
        let late_answer = async {
            match waiting.as_mut() {
                Some(answer) => {
                    answer.as_mut().await;
                }
                None => future::pending::<()>().await,
            }
        };
        let more = future::select(feedback.next(), Box::pin(late_answer));
        let event = match future::select(result.as_mut(), more).await {
            Either::Left((r, _)) => Event::Done(r),
            Either::Right((Either::Left((Some(()), _)), _)) => Event::Heard,
            Either::Right((Either::Left((None, _)), _)) => Event::FeedbackEnded,
            Either::Right((Either::Right(((), _)), _)) => Event::Answered,
        };
        match event {
            Event::Done(result) => {
                return Called::Done {
                    result,
                    answered: came(&mut waiting),
                }
            }
            Event::Heard => heard(),
            Event::Answered => waiting = None,
            Event::FeedbackEnded => {
                let result = result.await;
                return Called::Done {
                    result,
                    answered: came(&mut waiting),
                };
            }
        }
    }
}

/// The answer came: already noted, or there now (it may have come in the same turn as
/// the result).
fn came<A: Future>(waiting: &mut Option<Pin<Box<A>>>) -> bool {
    waiting
        .as_mut()
        .is_none_or(|answer| answer.as_mut().now_or_never().is_some())
}

#[cfg(test)]
mod tests {
    use super::*;
    use futures::channel::{mpsc, oneshot};
    use futures::executor::block_on;
    use std::cell::Cell;

    /// A result future that resolves to whatever is sent on `tx`.
    async fn result_of(rx: oneshot::Receiver<u32>) -> u32 {
        rx.await.unwrap_or(0)
    }

    #[test]
    fn answered_then_feedback_then_result() {
        let (answer_tx, answer_rx) = oneshot::channel();
        let (result_tx, result_rx) = oneshot::channel();
        let (feedback_tx, feedback_rx) = mpsc::unbounded();
        let heard = Cell::new(0);
        answer_tx.send(Some(result_of(result_rx))).ok();
        feedback_tx.unbounded_send(()).unwrap();
        feedback_tx.unbounded_send(()).unwrap();
        result_tx.send(7).unwrap();
        let called = block_on(follow_call(
            async { answer_rx.await.ok().flatten() },
            feedback_rx,
            || -> future::Ready<u32> { panic!("asked by id though the answer came") },
            || heard.set(heard.get() + 1),
        ));
        assert_eq!(
            called,
            Called::Done {
                result: 7,
                answered: true
            }
        );
    }

    /// TODO 240: the answer never comes, but the goal's feedback does. Each feedback
    /// counts as word from the follower, and the result comes by the goal's id.
    #[test]
    fn feedback_without_an_answer_keeps_the_follower_alive() {
        let (_answer_tx, answer_rx) = oneshot::channel::<Option<future::Ready<u32>>>();
        let (by_id_tx, by_id_rx) = oneshot::channel();
        let by_id_tx = std::cell::RefCell::new(Some(by_id_tx));
        let (feedback_tx, feedback_rx) = mpsc::unbounded();
        let heard = Cell::new(0);
        let asked = Cell::new(0);
        for _ in 0..3 {
            feedback_tx.unbounded_send(()).unwrap();
        }
        // The result by id comes only after all three feedbacks have been heard.
        let feedback = feedback_rx.inspect(|()| {
            if heard.get() == 2 {
                if let Some(tx) = by_id_tx_cell(&by_id_tx) {
                    tx.send(9).ok();
                }
            }
        });
        let called = block_on(follow_call(
            async { answer_rx.await.ok().flatten() },
            feedback,
            || {
                asked.set(asked.get() + 1);
                result_of(by_id_rx)
            },
            || heard.set(heard.get() + 1),
        ));
        assert_eq!(
            called,
            Called::Done {
                result: 9,
                answered: false
            }
        );
        assert_eq!(heard.get(), 3, "every feedback is word from the follower");
        assert_eq!(asked.get(), 1, "the result asked for by id once");
    }

    /// Takes the sender out of its cell once.
    fn by_id_tx_cell(
        cell: &std::cell::RefCell<Option<oneshot::Sender<u32>>>,
    ) -> Option<oneshot::Sender<u32>> {
        cell.borrow_mut().take()
    }

    #[test]
    fn refused() {
        let (feedback_tx, feedback_rx) = mpsc::unbounded::<()>();
        let called: Called<u32> = block_on(follow_call(
            async { None::<future::Ready<u32>> },
            feedback_rx,
            || -> future::Ready<u32> { panic!("asked by id though refused") },
            || panic!("heard though refused"),
        ));
        drop(feedback_tx);
        assert_eq!(called, Called::Refused);
    }

    #[test]
    fn feedback_first_then_a_late_answer() {
        let (answer_tx, answer_rx) = oneshot::channel();
        let (by_id_tx, by_id_rx) = oneshot::channel();
        let (feedback_tx, feedback_rx) = mpsc::unbounded();
        feedback_tx.unbounded_send(()).unwrap();
        let answer_tx = std::cell::RefCell::new(Some(answer_tx));
        let by_id_tx = std::cell::RefCell::new(Some(by_id_tx));
        let heard = Cell::new(0);
        // After the first feedback: the answer; after the second: the result.
        let feedback = feedback_rx.inspect(|()| match heard.get() {
            0 => {
                if let Some(tx) = answer_tx.borrow_mut().take() {
                    tx.send(Some(future::ready(1))).ok();
                }
                feedback_tx.unbounded_send(()).unwrap();
            }
            _ => {
                if let Some(tx) = by_id_tx_cell(&by_id_tx) {
                    tx.send(5).ok();
                }
            }
        });
        let called = block_on(follow_call(
            async { answer_rx.await.ok().flatten() },
            feedback,
            || result_of(by_id_rx),
            || heard.set(heard.get() + 1),
        ));
        assert_eq!(
            called,
            Called::Done {
                result: 5,
                answered: true
            }
        );
        assert_eq!(heard.get(), 2);
    }

    #[test]
    fn no_feedback_at_all_waits_for_the_answer() {
        let (feedback_tx, feedback_rx) = mpsc::unbounded::<()>();
        drop(feedback_tx);
        let called = block_on(follow_call(
            async { Some(future::ready(3)) },
            feedback_rx,
            || -> future::Ready<u32> { panic!("asked by id with no feedback") },
            || panic!("heard with no feedback"),
        ));
        assert_eq!(
            called,
            Called::Done {
                result: 3,
                answered: true
            }
        );
    }

    #[test]
    fn feedback_ending_leaves_the_result() {
        let (answer_tx, answer_rx) = oneshot::channel();
        let (result_tx, result_rx) = oneshot::channel();
        let (feedback_tx, feedback_rx) = mpsc::unbounded();
        answer_tx.send(Some(result_of(result_rx))).ok();
        feedback_tx.unbounded_send(()).unwrap();
        drop(feedback_tx);
        let heard = Cell::new(0);
        let result_tx = std::cell::RefCell::new(Some(result_tx));
        let called = block_on(follow_call(
            async { answer_rx.await.ok().flatten() },
            feedback_rx.inspect(|()| {
                if let Some(tx) = by_id_tx_cell(&result_tx) {
                    tx.send(4).ok();
                }
            }),
            || -> future::Ready<u32> { panic!("asked by id though the answer came") },
            || heard.set(heard.get() + 1),
        ));
        assert_eq!(
            called,
            Called::Done {
                result: 4,
                answered: true
            }
        );
        assert_eq!(heard.get(), 1);
    }
}
