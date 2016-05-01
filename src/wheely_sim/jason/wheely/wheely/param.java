package wheely;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.InternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;
import jason.asSyntax.StringTerm;
import jason.asSyntax.Atom;

import java.util.concurrent.ThreadLocalRandom;

/**

  <p>Internal action: <b><code>wheely.param(name,P)</code></b>.
  
  <p>Description: randomises a param, based on named constraints.

  <p>Parameters:<ul>

  <li>- name (string): name of the param for constraint selection.</li>  

  <li>- P (number): randomly generated output based on constraints.</li>
  
  </ul>
  
  <p>Examples:<ul> 

  <li> <code>wheely.param(waittime,P)</code>: unifies P with a random param chosen using the constraints for 'waittime'.</li>

  </ul>
  
 */
public class param extends DefaultInternalAction {
    
    private static InternalAction singleton = null;
    public static InternalAction create() {
        if (singleton == null) 
            singleton = new param();
        return singleton;
    }
    
    @Override public int getMinArgs() { return 2; }
    @Override public int getMaxArgs() { return 2; }

    /*
light_response_time = [0,100]
light_crossing_time = [100,5500]
gaze_wait_time      = [0,50]
gaze_wait_time_2    = [0,50]
test_end_time       = [10000,10000]
new_cmd_wait_time   = [0,4000]
user_commands       = [0,2.5]
crossing_signals    = [0,2]
gaze_sensor         = [0.7,1.0]
    */

    private long uniform(long a, long b) {
	return ThreadLocalRandom.current().nextLong(a, b+1);
    }

    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        checkArguments(args);

	double value = 0;

	String name = "";

	if (args[0].isString()) {
	    name = ((StringTerm)args[0]).getString();
	} else if (args[0].isAtom()) {
	    name = ((Atom)args[0]).getFunctor();
	}

	switch (name) {
	    case "light_response_time":
		value = (double)uniform(0, 100);
		break;
	    case "light_crossing_time":
		value = (double)uniform(100, 5500);
		break;
	    case "gaze_wait_time":
		value = (double)uniform(0, 50);
		break;
	    case "gaze_wait_time_2":
		value = (double)uniform(0, 50);
		break;
	    case "test_end_time":
		value = (double)uniform(10000, 10000);
		break;
	    case "new_cmd_wait_time":
		value = (double)uniform(0, 4000);
		break;
	    case "user_commands":
		value = (double)uniform(0, 2);
		break;
	    case "crossing_signals":
		value = (double)uniform(0, 1);
		break;
	    case "gaze_sensor":
		value = ThreadLocalRandom.current().nextDouble(0.7,1.0);
		break;
	}

	return un.unifies(args[1], new NumberTermImpl(value));
    }
}
