// Agent wheely in project wheely.mas2j

/* Initial beliefs and rules */
location(a).

/* Initial goals */


/* Plans */

// If we desire to be where we are, done!
@l0
+!location(Y)
	: location(Y)
	<- .print("Already at our destination.").

// If we desire to be elsewhere, and the traffic is moving, need to wait.
@l1
+!location(Y)
	: not location(Y) & not traffic(red)
	<- .print("Asking lights to stop traffic.");
	+wantToCross(Y);
	.send(lights,achieve,traffic(red)).
	
// If we desire to be elsewhere, and the traffic is stopped, move.
@l2
+!location(Y)
	: not location(Y)
	<- ?traffic(red); 
	.print("Crossing...");
	-+location(Y);
	.print("Now at location ", Y).

// When we come to learn that the traffic has been stopped, we can do goal
@red
+traffic(red)[source(lights)]
	: wantToCross(Y)
	<- .print("Traffic has stopped for us!");
	!location(Y);
	-wantToCross(Y).
