// Agent wheely in project wheely.mas2j

/* Initial beliefs and rules */
location(0).
gaze(0.0).

/* Initial goals */


/* Plans */

@printgaze
+gaze(C)
	<- .print("Received gaze of ", C).

@printloc
+location(L)
	<- .broadcast(untell,location(_));
	.print("Now at location: ", L);
	.broadcast(tell,location(L)).

@lerror
+!location(Y)
	: not .number(Y)
	<- .print("Bad location: ", Y);
	.fail.

// If we desire to be where we are, done!
//@l0
//+!location(Y)
//	: location(Y)
//	<- .print("Already at our destination.").
	

	
// New move command, but we are on the road, need to wait for the current move to tell us where we are
@linterrupt_red
+!location(Y)
	: inTransit(_) & traffic(red)
	<- .print("New move goal interrupting current move.");
	+interrupt;
	.wait("+location(P)", 1000);
	-interrupt;
	-inTransit(_);
	.print("Unblocked, got progress of: ", P);
	!location(Y).

	
// If we desire to be elsewhere, and the traffic is moving, need to wait.
@l1
+!location(Y)
//	: not location(Y) & not traffic(red)
	: not traffic(red)
	<- .print("Asking lights to stop traffic.");
	.send(lights,achieve,traffic(red));
	.wait(traffic(red),20000);
	!location(Y).
	
// If we desire to be elsewhere, and the traffic is stopped, move.
@l2
+!location(Y)
//	: not location(Y)
	: traffic(red) & gaze(C) & C >= 0.8
	<- .print("Crossing...");
	+inTransit(Y);
	.wait("+interrupt",6000,Elapsed);
	if (Elapsed < 6000) {
		// We were interrupted, cancel.
		.print("Move command interrupted.");
		// Need to know how far we've gone.
		Pcnt_prog = Elapsed / 6000;
		if (Y == 0) {
			Loc_partial = 1 - Pcnt_prog;
		} else {
			Loc_partial = Pcnt_prog;
		};
		-+location(Loc_partial);
	} else {
		// Uninterrupted, location goal completed.
		.print("Move completed uninterrupted.");
		-+location(Y);
		-inTransit(_);
	}.

// Traffic stopped but gaze not detected	
@l3
+!location(Y)
	: traffic(red)
	<- .print("Gaze not detected!");
	.wait(gaze(C) & C >= 0.8, 20000);
	!location(Y).
	
@green
+traffic(green)[source(lights)]
	: inTransit(_)
	<- .print("Lights changed whilst on road.");
	+interrupt;
	.wait("+location(P)", 1000);
	-interrupt;
//	-inTransit(_);
	.print("Unblocked, got progress of: ", P);
	if (P < 0.4) {
		// Haven't gone far enough, turn back.
		.print("Turning back to 0...");
		-inTransit(_);
		-+location(0); // Can ignore the time here, not important for testing?
		//!location(0);
	} else {
		if (P > 0.6) {
			// Too close to 1, turn back (or perhaps carry on)
			.print("Turning back to 1...");
			-inTransit(_);
			-+location(1);
			//!location(1);
		} else {
			// Far enough to finish original order.
			?inTransit(O);
			-inTransit(_);
			.print("Pushing on to original dest: ", O);
			-+location(0);
			//!location(O);
		};
	}.
