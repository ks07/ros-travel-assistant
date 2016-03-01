package wheely;

import jason.*;
import jason.asSemantics.*;
import jason.asSyntax.*;

import java.io.IOException;
import java.io.PrintWriter;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.logging.Logger;

/**
  <p>Internal action: <b><code>wheely.save</code></b>.
  
  <p>Description: used for saving messages to a file.

*/
public class save extends DefaultInternalAction {
	
	private final ArrayList<String> toSave = new ArrayList<>();
	private final Path outPath;

    private static InternalAction singleton = null;
    public static InternalAction create() {
        if (singleton == null) 
            singleton = new save();
        return singleton;
    }
	private save() {
		outPath = Paths.get("bdi_test.txt");
	}
    
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        String sout = argsToString(args);
		// Agent ag = ts.getAg();
        
//        if (ts != null) {
//            saveLine(sout.toString());
//        } else {
//            saveLine(sout.toString());
//        }
		toSave.add(sout.toString());
        
        return true;
    }
	
	private void saveLine(String line) {
		try {
			Files.write(outPath, Arrays.asList(line), Charset.forName("UTF-8"));
		} catch (IOException ioe) {
			System.out.println("fail");	
		}
	}

    protected String argsToString(Term[] args) {
        StringBuilder sout = new StringBuilder();
        
        for (int i = 0; i < args.length; i++) {
            if (args[i].isString()) {
                StringTerm st = (StringTerm)args[i];
                sout.append(st.getString());
            } else {
                Term t = args[i];
                if (! t.isVar()) {
                    sout.append(t);
                } else {
                    sout.append(t+"<no-value>");
                }
            }
        }
        return sout.toString();
    }
	
	@Override
	public void destroy() throws Exception {
		try {
			Files.write(outPath, toSave, Charset.forName("UTF-8"));
		} catch (IOException ioe) {
			System.out.println("Test saving failed: " + ioe.getMessage());
		}
	}
}
