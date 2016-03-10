package wheely;

import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.InternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.NumberTermImpl;
import jason.asSyntax.Term;

import java.lang.System;

/**

  <p>Internal action: <b><code>wheely.unixtime(T)</code></b>.
  
  <p>Description: gets the current unixtime, in milliseconds.

  <p>Parameters:<ul>
  
  <li>- unixtime (number): the current unixtime in milliseconds.</li>
  
  </ul>
  
  <p>Examples:<ul> 

  <li> <code>wheely.unixtime(T)</code>: unifies T with the current unixtime.</li>

  </ul>

  @see jason.stdlib.time
  
 */
public class unixtime extends DefaultInternalAction {
    
    private static InternalAction singleton = null;
    public static InternalAction create() {
        if (singleton == null) 
            singleton = new unixtime();
        return singleton;
    }
    
    @Override public int getMinArgs() { return 1; }
    @Override public int getMaxArgs() { return 1; }

    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        checkArguments(args);

	long timenow = System.currentTimeMillis();
        return un.unifies(args[0], new NumberTermImpl(timenow));
    }
}
