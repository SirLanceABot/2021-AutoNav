package frc.robot;

import java.lang.invoke.MethodHandles;

public class Test 
{
    private static int testCounter = 0;
    private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
    
    public Test()
    {
        System.out.println(Constants.CONSTRUCTOR_STARTING + fullClassName);
        // Add your constructor code here

        if(testCounter >= Constants.TEST_LIMIT)
        {
            System.out.println("ERROR -- Attempted to create too many objects: " + fullClassName);
        }
        else
        {
            testCounter++;
        }

        System.out.println(Constants.CONSTRUCTOR_FINISHING + fullClassName);
    }

    public void init()
    {

    }

    public void periodic()
    {

    }

    public void end()
    {

    }
    
}
