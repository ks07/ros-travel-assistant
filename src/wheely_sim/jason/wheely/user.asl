// Agent user in project wheely.mas2j

/* Initial beliefs and rules */
location(0).

/* Initial goals */

!start.

/* Plans */

+!start
	: true
	<- wheely.param(light_response_time,P);
	.print("Got param: ", P);
	for ( .member(DOH,[1,0,1,0,1,0,1]) ) {
		wheely.param(user_commands,D);
		!location(D);
		wheely.param(new_cmd_wait_time,T);
		.wait(T)
		//wheely.save("paramdelay,newcmdwait")
	};
	wheely.save("paramdelay,testend");
	wheely.save("pub,user_commands,127").

@l1
+!location(L)
	: not location(L)
	<- wheely.save("pub,user_commands,",L);
	.send(wheely,achieve,location(L));
	wheely.param(gaze_wait_time,GWT);
	.wait(GWT);
	.broadcast(untell,gaze(_));
	//wheely.save("paramdelay,gazewait");
	wheely.param(gaze_sensor,GS);
	.broadcast(tell,gaze(GS));
	wheely.save("pub,gaze_sensor,",GS);
	//.wait(10000);
	//.send(wheely,askOne,location(N),location(N));
	//-+location(N);
	//?location(L);
	//.print("Made it to ",L);
	wheely.param(gaze_wait_time_2,GWTT);
	//wheely.save("paramdelay,gazewait2");
	.wait(GWTT);
	.broadcast(untell,gaze(_));
	.print("untell");
	wheely.param(gaze_sensor,GSN);
	.broadcast(tell,gaze(GSN));
	wheely.save("pub,gaze_sensor,",GSN).

@lsame
+!location(L)
	: location(L)
	<- ?location(L);
	wheely.save("pub,user_commands,",L);
	.print("Already at ",L).
