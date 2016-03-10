// Agent user in project wheely.mas2j

/* Initial beliefs and rules */

/* Initial goals */

!start.

/* Plans */

+!start
	: true
	<- wheely.save("pub,user_commands,1");
	.send(wheely,achieve,location(1)).

