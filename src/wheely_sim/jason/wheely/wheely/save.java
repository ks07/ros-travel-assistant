package wheely;

import jason.*;
import jason.asSemantics.*;
import jason.asSyntax.*;

import java.io.IOException;
import java.io.PrintWriter;
import java.lang.System;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.logging.Logger;
import java.util.Map;

/**
   <p>Internal action: <b><code>wheely.save</code></b>.
  
   <p>Description: used for saving messages to a file.

*/
public class save extends DefaultInternalAction {
	
    private final HashMap<String, List<String>> toSave = new HashMap<>();
    private static final String outDir = "bdi_tests";

    private static InternalAction singleton = null;
    public static InternalAction create() {
        if (singleton == null) 
            singleton = new save();
        return singleton;
    }
    private save() {
    }
    
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        String sout = argsToString(args);
	String key = "anon"; 
        
        if (ts != null) {
	    Agent ag = ts.getAg();
	    key = ts.getUserAgArch().getAgName();
	}

	if (!toSave.containsKey(key)) {
	    toSave.put(key, new ArrayList<String>());
	}
	long timenow = System.currentTimeMillis();
	toSave.get(key).add(Long.toString(timenow) + "," + sout);
	ts.getLogger().info("Saving: " + key + " : " + sout);
        
        return true;
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
	for (Map.Entry<String,List<String>> entry : toSave.entrySet()) {
	    try {
		Path outPath = Paths.get(outDir, entry.getKey().replaceAll("[^-_.A-Za-z0-9]","_").concat(".txt"));
		Files.createDirectories(outPath.getParent());
		Files.write(outPath, entry.getValue(), Charset.forName("UTF-8"));
	    } catch (IOException ioe) {
		System.out.println("Test saving failed (" + entry.getKey() + "): " + ioe.toString());
	    }
	}
    }
}
