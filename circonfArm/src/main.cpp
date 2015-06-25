// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// A tutorial on how to use the Cartesian Interface to control a limb
// in the operational space.
//
// Author: Raffaello Camoriano - <raffaello.camoriano@iit.it>
// Based on the example module by Ugo Pattacini - <ugo.pattacini@iit.it>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>

#include <yarp/dev/Drivers.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <gsl/gsl_math.h>

#include <stdio.h>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;


class CtrlThread: public RateThread,
                  public CartesianEvent
{
protected:
    PolyDriver         client;
    ICartesianControl *icart;
    
    int mode;           // Mode: 1: XY; 2: XZ; 3: YZ (plane of the circumference)
    Vector center;      // Center of the circumference
    double radius;      // Radius of the circumference
    double frequency;   // Frequency of the movement

    Vector xd;
    Vector od;

    int startup_context_id;

    double t;
    double t0;
    double t1;

    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback()
    {
        fprintf(stdout,"20%% of trajectory attained\n");
    }

public:
    CtrlThread(const double period , const Vector center_ ,const double radius_ , const double frequency_ , const int mode_) : RateThread(int(period*1000.0))
    {
        // Set thread parameters
        mode = mode_;
        center = center_;
        radius = radius_;
        frequency = frequency_;
        
        // we wanna raise an event each time the arm is at 20%
        // of the trajectory (or 70% far from the target)
        cartesianEventParameters.type="motion-ongoing";
        cartesianEventParameters.motionOngoingCheckPoint=0.2;
    }

    virtual bool threadInit()
    {
        // open a client interface to connect to the cartesian server of the simulator
        // we suppose that:
        //
        // 1 - the iCub simulator is running
        //     (launch iCub_SIM)
        //
        // 2 - the cartesian server is running
        //     (launch simCartesianControl)
        //     
        // 3 - the cartesian solver for the left arm is running too
        //     (launch iKinCartesianSolver --context simCartesianControl --part left_arm)
        //
        Property option("(device cartesiancontrollerclient)");
        option.put("remote","/icubSim/cartesianController/left_arm");
        option.put("local","/cartesian_client/left_arm");

        if (!client.open(option))
            return false;

        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // Disable torso DoFs
        newDof[0]=0;
        newDof[1]=0;
        newDof[2]=0;

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);

        xd.resize(3);
        od.resize(4);

        return true;
    }

    virtual void afterStart(bool s)
    {
        if (s)
            fprintf(stdout,"Thread started successfully\n");
        else
            fprintf(stdout,"Thread did not start\n");

        t=t0=t1=Time::now();
    }

    virtual void run()
    {
        t=Time::now();

        generateTarget();

        // go to the target :)
        // (in streaming)
        icart->goToPose(xd,od);

        // some verbosity
        printStatus();
    }

    virtual void threadRelease()
    {    
        // we require an immediate stop
        // before closing the client for safety reason
        icart->stopControl();

        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);

        client.close();
    }

    void generateTarget()
    {   
      
        // translational target part: a circular trajectory
      
// 	if (m/*ode == 0)
// 	{*/
	  
	//Mode: 0: XY; 1: YZ; 2: ZX
      
	xd[mode % 3] = center[mode % 3]+radius*sin(2.0*M_PI*frequency*(t-t0));
	xd[mode+1 % 3] = center[mode+1 % 3]+radius*cos(2.0*M_PI*frequency*(t-t0));
	xd[mode+2 % 3] = center[mode+2 % 3];
	
// 	}
// 	else if (mode == 1)
// 	{
// 	  // YZ plane
// 	  xd[0]=center[0];
// 	  xd[1]=center[1]+radius*cos(2.0*M_PI*frequency*(t-t0));
// 	  xd[2]=center[2]+radius*sin(2.0*M_PI*frequency*(t-t0));            
// 	}
// 	else if (mode == 2)
// 	{
// 	  // XZ plane
// 	  xd[0]=center[0]+radius*cos(2.0*M_PI*frequency*(t-t0));
// 	  xd[1]=center[1];
// 	  xd[2]=center[2]+radius*sin(2.0*M_PI*frequency*(t-t0));            
// 	}

    
        // we keep the orientation of the left arm constant:
        // we want the middle finger to point forward (end-effector x-axis)
        // with the palm turned down (end-effector y-axis points leftward);
        // to achieve that it is enough to rotate the root frame of pi around z-axis
        od[0]=0.0; od[1]=0.0; od[2]=1.0; od[3]=M_PI;
    }

    void printStatus()
    {        
        if (t-t1>=PRINT_STATUS_PER)
        {
            Vector x,o,xdhat,odhat,qdhat;

            // we get the current arm pose in the
            // operational space
            icart->getPose(x,o);

            // we get the final destination of the arm
            // as found by the solver: it differs a bit
            // from the desired pose according to the tolerances
            icart->getDesired(xdhat,odhat,qdhat);

            double e_x=norm(xdhat-x);
            double e_o=norm(odhat-o);

            fprintf(stdout,"+++++++++\n");
            fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
            fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
            fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
            fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
            fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
            fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
            fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
            fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
            fprintf(stdout,"---------\n\n");

            t1=t;
        }
    }
};



class CtrlModule: public RFModule
{
protected:
    CtrlThread *thr;

public:
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        // Get parameters from conf file
        Vector center_;
        center_.resize(3);
        center_[0] = rf.find("ctr_x").asDouble();
        center_[1] = rf.find("ctr_y").asDouble();
        center_[2] = rf.find("ctr_z").asDouble();
        const double radius_ = rf.find("radius").asDouble();
        const double frequency_ = rf.find("frequency").asDouble();
        const int mode_ = rf.find("mode").asInt();
        
        thr=new CtrlThread(CTRL_THREAD_PER , center_ ,radius_ , frequency_ , mode_);
        if (!thr->start())
        {
            delete thr;
            return false;
        }

        return true;
    }

    virtual bool close()
    {
        thr->stop();
        delete thr;

        return true;
    }

    virtual double getPeriod()    { return 1.0;  }
    virtual bool   updateModule() { return true; }
};



int main(int argc, char *argv[])
{   
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"Error: yarp server does not seem available\n");
        return -1;
    }

    CtrlModule mod;

    ResourceFinder rf;
    
    rf.setVerbose(true);
    rf.setDefaultConfigFile("circonfArm_config.ini");
    rf.setDefaultContext("circonfArm");
    rf.setDefault("name","circonfArm");
    rf.configure(argc,argv);
    return mod.runModule(rf);
}



