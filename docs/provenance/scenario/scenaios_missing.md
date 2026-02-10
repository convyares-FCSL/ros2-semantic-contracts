Gemini said
To achieve 100% Semantic Coverage—meaning every "MUST", "MUST NOT", and "REJECT" in your specifications is backed by an executable test—you need to add 9 new scenarios.

These are categorized by priority: Critical Safety (Tier 1), Protocol Strictness (Tier 2), and Parity Breakers (Tier 3).

Tier 1: Critical Safety & Negative Controls
These scenarios close the "Safety Gap" where a backend could currently fail dangerously (data corruption, zombies, race conditions) and still pass the suite.

Scenario ID	Title	Spec Target	The Check
A15	Goal Identity Collision	SPEC_AC01	
Input: Send a new goal using the UUID of a recently completed goal.


Expected: REJECT (or treat as duplicate request).


Failure: Server spawns a new thread or corrupts the old goal record.

L14	Lifecycle Re-entrancy	SPEC_LC04	
Input: Trigger a transition (e.g., activate) from inside a transition callback (on_configure).


Expected: REJECT (Busy) or BLOCK (Deadlock detection).


Failure: Undefined state transition or state corruption.

P36	Range Constraint Enforcement	SPEC_PC10	
Input: Declare a float with range [0.0, 1.0]. Attempt to set value 5.0.


Expected: REJECT (Validation Failure).


Failure: Value is updated to 5.0, violating the descriptor contract.

S21	Zombie Resource Ping	SPEC_SYS05	
Input: Kill a node. Immediately attempt to ping its service/action server from another node.


Expected: FAIL (Connection Refused / Not Found) within a tight bound.


Failure: Service hangs or appears "available" (Zombie state).

Tier 2: Protocol Strictness & Edge Cases
These scenarios ensure strict adherence to the API contract, preventing "sloppy" implementations that work by accident.

Scenario ID	Title	Spec Target	The Check
A18	Premature Result Request	SPEC_AC07	
Input: Call get_result on a goal that is currently Executing.


Expected: BLOCK (Yield) or PENDING.


Failure: Returns partial/empty result or error before terminal state is reached.

L15	Inactive Traffic Leak	SPEC_L07	
Input: Publish to a lifecycle-managed topic while the node is Inactive.


Expected: SILENCE (No messages received by subscriber).


Failure: Data leaks through despite the node being "Inactive".

P37	Ambiguous Declaration	SPEC_PC03	
Input: Call declare_parameter with no type and no default value.


Expected: REJECT (Ambiguous).


Failure: Node infers a random type or defaults to NOT_SET in a valid state.

S22	Namespace Isolation	SPEC_SYS01	
Input: Node A attempts to set_parameters on Node B without permission/visibility constraints.


Expected: REJECT (Unauthorized/Hidden).


Failure: Cross-node state corruption is allowed.

Tier 3: Parity Breakers (Concurrency & Deadlocks)
These scenarios are designed to expose the known architectural differences between rclcpp (threaded) and rclpy (GIL/async), which current sequential tests miss.

Scenario ID	Title	Spec Target	The Check
P38	Async Set Callback Reentrancy	SPEC_PC16	
Input: Inside a parameter validation callback, trigger a get_parameter call for a different key.


Expected: SUCCESS (Re-entrant read allowed).


Failure: Deadlock (rclcpp mutex starvation) or Recursion Error.

Coverage Matrix Summary
Adding these 9 scenarios completes the coverage map:

Safety/Identity: Covered by A15 & S22.

State Machine Integrity: Covered by L14 & A18.

Data Integrity: Covered by P36 & P37.

Resource Hygiene: Covered by S21 & L15.

Concurrency Architecture: Covered by P38.

Recommendation: Implement Tier 1 (A15, L14, P36, S21) first. These represent the "Critical Safety" layer missing from the current suite.