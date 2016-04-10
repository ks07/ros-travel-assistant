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
	};
	wheely.save("paramdelay,testend");
	wheely.save("pub,user_commands,127").

@l1
+!location(L)
	: not location(L)
	<- wheely.save("pub,user_commands,",L);
	.send(wheely,achieve,location(L));
	.wait(2000);
	.broadcast(untell,gaze(_));
	wheely.save("paramdelay,gazewait");
	.broadcast(tell,gaze(0.85));
	wheely.save("pub,gaze_sensor,0.85");
	.wait(10000);
	.send(wheely,askOne,location(N),location(N));
	-+location(N);
	?location(L);
	.print("Made it to ",L);
	.broadcast(untell,gaze(_));
	wheely.save("paramdelay,gazewait2");
	.broadcast(tell,gaze(0.4));
	wheely.save("pub,gaze_sensor,0.4").

@lsame
+!location(L)
	: location(L)
	<- ?location(L);
	.print("Already at ",L).
