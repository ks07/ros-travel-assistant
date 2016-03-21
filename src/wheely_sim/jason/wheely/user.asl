// Agent user in project wheely.mas2j

/* Initial beliefs and rules */
location(0).

/* Initial goals */

!start.

/* Plans */

+!start
	: true
	<- for ( .member(D,[1,0,1,0,1,0,1]) ) {
		!location(D);
	}.

@l1
+!location(L)
	: not location(L)
	<- wheely.save("pub,user_commands,",L);
	.send(wheely,achieve,location(L));
	.wait(10000);
	.send(wheely,askOne,location(N),location(N));
	-+location(N);
	?location(L);
	.print("Made it to ",L).

@lsame
+!location(L)
	: location(L)
	<- ?location(L);
	.print("Already at ",L).
