// Agent lights in project wheely.mas2j

/* Initial beliefs and rules */

traffic(green).

/* Initial goals */

/* Plans */

@t0
+!traffic(C) : traffic(D) & C \== D
	<- wheely.save("waitfor,light_commands,1");
	wheely.param(light_response_time,W);
	//.random(R);
	//W = R * 2000;
	.wait(W);
	//wheely.save("paramdelay,light_response_time");
	-+traffic(C);
	?traffic(E);
	.print("My lights are now ", E);
	!traffic_timeout(E).
	
@ttr
+!traffic_timeout(red)
	: true
	<- //.wait(8000);
	wheely.param(light_crossing_time,T);
	.wait(T);
	//wheely.save("paramdelay,light_crossing_time");
	wheely.param(crossing_signals,CS);
	-+traffic(green);
	.print("Lights switched back to green.").
	
@tt_
+!traffic_timeout(_)
	: true
	<- true.

@t1
+!traffic(C) <- .print("My lights are already ", C).

@change
+traffic(C)
	: true
	<- .broadcast(untell,traffic(_));
	.print("Broadcasting ", traffic(C)); // This is necessary, for some reason - timing?
	.broadcast(tell,traffic(C));
	wheely.param(crossing_signals,CS);
	if (C == green) {
		wheely.save("pub,crossing_signals,",CS); //0
	} else {
		wheely.save("pub,crossing_signals,",CS); //1
	}.

