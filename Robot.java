package coverage;

import java.util.*;

public class Robot
{
    public Robot()
    {}

    //universal identifier
    int id; //start from 0

    // Physical properties
    public point2 position; //robot position
    double heading;
    double speed;
    double rightSpeed;  //not used now, but in the future, robot might have different wheel speed
    double width;
    
    //sensor FOV 
    double sensor_heading = 0;
    point2 FOV_edge_point_left = new point2();   //A point on the left edge of FOV cone. Distance to robot is sensing range
    point2 FOV_edge_point_right = new point2();
    point2 FOV_edge_impact_point_left = new point2(); //the point where left edge hits something
    point2 FOV_edge_impact_point_right = new point2();
    double FOV_degree = Math.PI*2; //was Math.PI/3*2 = 120 degree
    double FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    double FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    boolean is_FOV_constrained = true;  
    double dFdh; //partial derivative w.r.t sensor heading 
    double sensor_turning_speed;
    double max_sensor_turning_speed = 0.1; //was 0.01
	
    public double robotMaxSpeed;

    //Simulation related
    double lastStatesUpdateTime;
    Simulation sim;

    //Sensing model
    public double sensingDecayFactor;

    //Control related
    public double controlPeriod;
    double lastControlTime;

    double dFdx; //TODO: not sure if these two should be here
    double dFdy;
    double dFdx2;
    double dFdy2;
    public double norminalGradientMagnitude = 1;   //if it is zero, the "speed" of a node is fixed. If it is positive, the speed is proportional to the gradient magnitude

    //control parameters
    public double surface_integral_increment = 1; //0.25 is the C++ version value; This variable will be changed by UI
    public double sensingCutoffRange = 80; //was 57
    public double smallDetectionBoost = 1; //the boost for area currently with 10%-40% joint detection probability, 1 means no boost
    public double ultraLowDetectionBoost = 1; //0%-10%, 1 means no boost
    public double iteration = 0; // to identify the first time the the low boost is on and used for it.
    public String boostType = "Noboost"; 
    public int boosttype = 0; // 0 means no boost, 1 means selfboost, 2 means neighborboost, 3 means concaveboost   
    boolean Node_on=false;
    double indexNum=1; // In order to make a decreasing step
    point2 pathMax = new point2(0,0);
    point2 pathDirection = new point2(0,0);
    point2 lastSensorPosition = new point2(0,0);
    double mincos = 1;
    double boostDuration = 500000;
    double neighborGain = 500;
    double neighborPower = 3;
    double Power = 4;
    double PowerGain = 100;
    double PhiGain = 100;
    double PhiPower = 2;
    int complete_temperature_iterations = 0;
	double cooling_rate = 0.95;
	double initial_temperature = 20000;
	double temperature = initial_temperature;




    //Environment
    Obstacle boundary;
    ArrayList<Obstacle> obstacles;
    EventDensity density;

    //Neighbors
    ArrayList<Robot> neighbors = new ArrayList<Robot>();

    //Trajectory history
    public boolean recordTrajectory = false;
    int trajectoryHistoryIndex = 0;
    int TRAJECTORY_HISTORY_SIZE = 2000; //the size is tentative, might be too large
    double trajectoryHistoryX[] = new double[TRAJECTORY_HISTORY_SIZE];
    double trajectoryHistoryY[] = new double[TRAJECTORY_HISTORY_SIZE];

    //Communication
    public double costToBase = 0;
    //public double costToPredecessor;
    public Robot predecessor = null;
    public ArrayList<Robot> descendants = new ArrayList<Robot>();
    
    public double potential_costToBase;
    public Robot potential_predecessor;
    public ArrayList<Robot> potential_descendants = new ArrayList<Robot>();
    
    protected double ri; //amount of information collected by me
    protected double zi; //amount of information going through me
    public double communicationCostWeight = 0;  //0 means we do not worry about communication cost
    protected final int bitPerDetection = 1; //the alpha3 in Li's paper
    protected double cost_to_base_with_data_rate;  //costToBase times the data rate, which is related to the area a node can cover

    //Position estimation, asynchronous stuff
    //sensor heading should be estimated too
    public point2 estimatedPosition= new point2(0,0);
    public point2 localEstimatedPosition= new point2(0,0);
    public point2 lastPosition= new point2(0,0);
    public double lastHeading;
    public double lastSpeed;
    public double lastBroadcastTime;
    public double estimationErrorThreshold = 0; //if the position estimation error exceeds this threshold, actual position and heading should be broadcasted. Default is 0
    public boolean showRed = true;
    public  double K4 = 0;
    public boolean isFixedError = false;

    //event info collection stuff
    public double dataCollectionDecay = 0.09;
    public double dataCollectionWeight = 1000;

    //determine when to stop broadcasting due to small gradient
    protected boolean isMyGradientMagSmall = false;
    protected double SMALL_GRADIENT_THRESHOLD = 0;//0.000000000001;   //if it is zero, the broadcast will not stop due to small gradient
   

    
    //connectivity stuff
	public boolean is_upstream_connectivity_in_danger = false;
	public boolean is_stretching_upstream = false;
	
	//seeker stuff, move to the location of another robot
	boolean is_seeker_mode = false;  
	Robot seeker_target = null;
	double seeking_hit_range = 0;  //if set as 0, robot will never switch out of seeker mode by itself
	
	//map building
	public boolean inActiveMapBuildingMode = false;
	
    
    public Robot(int _id, point2 p, double h, double leftS, double rightS, double w, double robotMaxS, double lastStatesUpdateT,
                 double controlP, double lastControlT, Obstacle bndr, ArrayList<Obstacle> gswl, EventDensity dnst,
            Simulation s, double sensing_decay)
    {
        id = _id;
        position = p;
        heading = h;
        speed = leftS;
        rightSpeed = rightS;
        width = w;
        robotMaxSpeed = robotMaxS;
        lastStatesUpdateTime = lastStatesUpdateT;
        controlPeriod = controlP;
        lastControlTime = lastControlT;
        boundary = bndr;
        density = dnst;
        obstacles = gswl;
        dFdx = 0;
        dFdy = 0;
        sim = s;
        sensingDecayFactor = sensing_decay;
        
        this.sensor_heading = id; //separate starting sensor heading;
        set_FOV_edge_points();
        
    }

    void UpdateStates(double currentTime) //update position and heading based on the current speed and (currentTime-lastStatesUpdateTime), should be called at every simulation step
    {
        double updateInterval = currentTime - lastStatesUpdateTime;


//perfect vehicle handling
        
        //seeker mode
        if(this.is_seeker_mode == true)
        {
        	if(point2.Dist(this.position,this.seeker_target.position)<this.seeking_hit_range)
        	{
        		is_seeker_mode = false;
        		return;
        	}
        	heading = Math.atan2(this.seeker_target.position.y-position.y,this.seeker_target.position.x-position.x);
        	point2 tempPosition = new point2(position.x + speed * Math.cos(heading) * updateInterval,
                    position.y + speed * Math.sin(heading) * updateInterval);
        	
        	obstacle_sliding(updateInterval, tempPosition);
        	RecordTrajectory();
        	
        	        
        	lastStatesUpdateTime = currentTime;
        	return; //do not go to normal mode
        }
        
        //normal mode
        
        //the next robot position
//        point2 tempPosition = new point2(position.x + 1/Math.pow(indexNum, 0.2)*speed * Math.cos(heading) * updateInterval,
//                                         position.y + 1/Math.pow(indexNum, 0.2)*speed * Math.sin(heading) * updateInterval);
//        
        point2 tempPosition = new point2(position.x + speed * Math.cos(heading) * updateInterval,
                                         position.y + speed * Math.sin(heading) * updateInterval);
        point2 prePosition = new point2(-1,-1);
        prePosition.copy(position);
        //prePosition is a copy of position, tempPposition is new updated position.
        // No boosting, Position is changed based on the gradient-based algorithm
        obstacle_sliding(updateInterval, tempPosition);

  
    	if(this.boosttype == 20)
		{
    		// simulated annealing
			if(Node_on)
			{
				
//				System.out.println("objective_previous is "+objective_previous);
				position.copy(prePosition);
				double objective_previous = sim.ObjectiveInfo();
			    point2 tempSAPosition = new point2(-2,-2);
				double xrange = 60;
				double yrange = 50;
				double xneighborrange = xrange/10;
				double yneighborrange = yrange/10;
				double xleft = Math.max(0,position.x-xneighborrange);
				double xright = Math.min(position.x+xneighborrange, xrange);
				double yleft = Math.max(0,position.y-yneighborrange);
				double yright = Math.min(position.y+yneighborrange, yrange);
				//System.out.println("xleft "+xleft+" xright "+xright +" yleft "+yleft + " yright "+yright);
				tempSAPosition.x = xleft + (xright-xleft)*Math.random();
				tempSAPosition.y = yleft + (yright-yleft)*Math.random();
			    obstacle_sliding(updateInterval,tempSAPosition);
	  
    		  double objective_temp = sim.ObjectiveInfo();
    		  double diff_objective;
//			  System.out.println("objective is "+objective_temp);
    		  if(objective_temp > objective_previous)
    		  {
    			  System.out.println("find a better solution");
    			 // position.copy(prePosition);
    		  
    		  }
    		  else
    		  {

			  diff_objective = objective_previous - objective_temp;
    		  System.out.println("diff_objective: "+diff_objective);
    		  System.out.println("real number: "+ Math.exp(-diff_objective/temperature));
    		  if(Math.random()>=Math.exp(-diff_objective/temperature))
    		  {
    			  // if Math.random()<Math.exp(-diff_objective/temperature)), accept the solution
    			  
/*    			  System.out.println("don't accept the solution");
    			  System.out.println("prePosition is: "+prePosition.x + " , "+ prePosition.y);
    			  System.out.println("Position is: "+position.x + " , "+ position.y);
    			  position.copy(prePosition);
    			  System.out.println("prePosition is: "+prePosition.x + " , "+ prePosition.y);
    			  System.out.println("Position is: "+position.x + " , "+ position.y);*/
    			  
    		  }
    		  

    	  }
    		  
    	  } 
  		  System.out.println("complete_temperature_iterations: "+complete_temperature_iterations);
  		System.out.println("temperature: "+ temperature);
  		  if(complete_temperature_iterations>10)
  		  {
  			  temperature = cooling_rate*temperature;
  			  
  			  complete_temperature_iterations = 0;
  		  }
			complete_temperature_iterations = complete_temperature_iterations + 1;
    }
      //obstacle_sliding(updateInterval, tempPosition);
      RecordTrajectory();
      
      this.modify_sensor_heading(updateInterval*this.sensor_turning_speed);

//end vehicle handling

       
        
        if (currentTime - lastControlTime >= controlPeriod)
        {
            sim.requestCount += (sim.realTimeRobotNumber - 1); //used for counting communication frequency
            SpeedControlAlgo(currentTime);
            lastControlTime = currentTime + Math.random() * 0.05 - 0.025;   //noisy control period
            //lastControlTime = currentTime;       //precise control period
        }
        
        lastStatesUpdateTime = currentTime;
    }
private void RecordTrajectory()
{
    if (this.recordTrajectory)
    {
        if (Math.abs(position.x - this.trajectoryHistoryX[trajectoryHistoryIndex]) +
            Math.abs(position.y - this.trajectoryHistoryY[trajectoryHistoryIndex]) > 2) //2 is recording threshold, adjustable
        {
            trajectoryHistoryIndex++;
            if(trajectoryHistoryIndex==TRAJECTORY_HISTORY_SIZE)
            {
                trajectoryHistoryIndex = 0; //start all over.
            }
            this.trajectoryHistoryX[trajectoryHistoryIndex] = position.x;
            this.trajectoryHistoryY[trajectoryHistoryIndex] = position.y;
        }
    }
}
    

		
	private void obstacle_sliding(double updateInterval, point2 tempPosition) {
		//detect if a robot runs into boundary or an obstacle P3 TODO: BSP tree to improve efficiency
		    boolean runIntoObstacle = false;
		    point2 direction = new point2(0, 0); //direction will be filled by LineOfSight
		   
		    //test if robot runs into boundary		    
		    if(tempPosition.x>boundary.largestX)
		    {
		    	tempPosition.x = boundary.largestX-0.001;
		    }
		    else if(tempPosition.x<boundary.smallestX)
		    {
		    	tempPosition.x = boundary.smallestX+0.001;
		    }
		    
		    if(tempPosition.y>boundary.largestY)
		    {
		    	tempPosition.y = boundary.largestY-0.001;
		    }
		    else if(tempPosition.y<boundary.smallestY)
		    {
		    	tempPosition.y = boundary.smallestY+0.001;
		    }	 

		    //if not into boundary, test if it runs into any obstacles, if it does, it will stop for one turn but the heading will be set properly   
		    for (int i1 = 0; (i1 < obstacles.size()) && (!runIntoObstacle); i1++)
		    {
		    	//TODO: there is a bug: if a node cross two boundaries of an obstacle, the direction of the
		    	//first one tested will be returned, not the closest's, a workaround is to describe obstacles
		    	//starting from the lower right corner.
		    	//TODO: another bug is that if a node cross boundaries of two obstacles
		        if (!(obstacles.get(i1).LineOfSight(position, tempPosition, direction)))
		        {
		            if (point2.dot(point2.minus(tempPosition, position), direction) >= 0)
		            {
		                heading = Math.atan2(direction.y, direction.x);                   
		            }
		            else
		            {
		                heading = Math.atan2( -direction.y, -direction.x);
		            }
		            runIntoObstacle = true;
		            tempPosition.copy(this.position); 
		            tempPosition.shift(speed * Math.cos(heading) * updateInterval,
		                    speed * Math.sin(heading) * updateInterval); 
		            
		            if(isPointInLOS(tempPosition))
		            {
		            	 this.position.copy(tempPosition);  
		            }
		            
		            //position.shift(speed * Math.cos(heading) * updateInterval, speed * Math.sin(heading) * updateInterval);	              
		            return;
		        }
		    }
		    
		    if (runIntoObstacle == false)
		    {
		        position.copy(tempPosition); 
		    }
	}

    void PrintRobotInfo()
    {
        System.out.print("robot " + id + " @ " + position.x + "," + position.y + "," + heading + "," +sensor_heading + "," + speed);
    }

    public boolean is_point_visible(point2 sample_point) //Notice that a point coincide with robot position is NOT visible
    {
    	double distance = point2.Dist(sample_point, position);
        
        if (distance > this.sensingCutoffRange || distance <= 0) //to make sure dist>0 is necessary. SamplePoint might coincide with robot position. dist=0 will cause divide by zero
        {	
        	return false;
        }
    	
        //boundary block all sensing. This test can be disabled if nodes can not stay out of boundary and boundary is always a rectangle.  
        if (!boundary.LineOfSight(position, sample_point)) return false;

        if(!isPointInFOV(sample_point))
        {
        	return false;
        }
        
        if(!isPointInLOS(sample_point))
        {
        	return false;
        }
              
    	return true;
    }
    
    public boolean isPointInLOS(point2 samplePoint) //in line of sight, not considering FOV, only obstacles
    {
    	 for (int i1 = 0; i1 < obstacles.size(); i1++)
         {
             if (!obstacles.get(i1).LineOfSight(position, samplePoint))
             {
                 return false;
             }
         }
    	 return true;
    }
    
    public boolean isPointInFOV(point2 samplePoint)
    {
        //If change is made here, make sure both FOV < 180 and >180 cases work correctly
        if(this.is_FOV_constrained && this.FOV_degree<2*Math.PI)
        {
        	if (this.FOV_degree<Math.PI)
        	{
        		if (point2.cross(point2.minus(FOV_edge_point_left, position), point2.minus(samplePoint,position))<0 
        				|| point2.cross(point2.minus(FOV_edge_point_right, position), point2.minus(samplePoint,position))>0)
        		{
        			return false;
        		}
        	}
        	else
        	{
        		if (point2.cross(point2.minus(FOV_edge_point_left, position), point2.minus(samplePoint,position))<0 
        				&& point2.cross(point2.minus(FOV_edge_point_right, position), point2.minus(samplePoint,position))>0)
        		{
        			return false;
        		}
        	}
        }
    
        return true;
    }
    
    synchronized void SpeedControlAlgo(double currentTime) //using control algorithm to set the robot's speed. Called every control period, not every simulation step
    {
    	if(this.is_FOV_constrained && this.FOV_degree<2*Math.PI)
    	{
    		FastSearchSensorHeading();
    	}
        //performance tuning:
        //MyTimer myTimer = new MyTimer();
        //long beginTime = myTimer.currentTimeMillis();
        dFdx = 0;
        dFdy = 0;
        ri = 0;
               
        double temp;

        double delta = surface_integral_increment; //0.25 is our original value
        point2 samplePoint = new point2( -1, -1);
        double neighborEffects;
        double dist;
        double alpha;
        double Phat, Phat0;

        double Alpha1;
        double Alpha2;
        double Beta1;
        double Beta2;
        double w1;
        double w2;
        double neighborDistanceSumx=0;
        double neighborDistanceSumy=0;
        double minNeighborDistance = 100000;
        int minNeighborindex = 100;
              
        double sample_upper_bound_x = Math.min(position.x+sensingCutoffRange, boundary.largestX);
        double sample_upper_bound_y = Math.min(position.y+sensingCutoffRange, boundary.largestY);
       
        
        
        //System.out.println("sensor id "+this.id+"postion: ("+ position.x +" " + position.y+ ")" );
        // change the sample point from Math.max(position.x-sensingCutoffRange,0.01) to (position.x-sensingCutoffRange,0).
        
        for (double horizontalSamplePoint =  Math.max(position.x-sensingCutoffRange, 0.01);horizontalSamplePoint<=sample_upper_bound_x; horizontalSamplePoint+=delta)
        {
            for (double verticalSamplePoint = Math.max(position.y-sensingCutoffRange, 0.01); verticalSamplePoint<=sample_upper_bound_y; verticalSamplePoint+=delta)
            {
                samplePoint.x = horizontalSamplePoint;
                samplePoint.y = verticalSamplePoint;
                dist = point2.Dist(samplePoint, position);
//                System.out.println("dist is: "+ dist);
                      
                if(is_point_visible(samplePoint))
                {
                	alpha = 1;
                }
                else
                {
                	alpha = EventDensity.WALL_DECAY_FACTOR;
                }
                
                if (alpha > 0) //performance concern, often EventDensity.WALL_DECAY_FACTOR is 0;
                {

                    //take care of ri along the way
                    ri += alpha * density.GetEventDensity(samplePoint, currentTime) * SensingModelFunction(dist);
                    // System.out.println("#"+id+"Node_on "+Node_on+" boosttype "+boosttype);
                   	
                    	neighborEffects = NeighborEffects(samplePoint);
                    	//System.out.println("samplePoint is " + samplePoint.x + " " + samplePoint.y + " position is "+position.x + " "+position.y + "neighborEffects is " + neighborEffects);
                    	w1 = density.GetEventDensity(samplePoint, currentTime) * alpha * (-SensingModelDerivative(dist)) * neighborEffects;
//                    	System.out.println("density.GetEventDensity(samplePoint, currentTime is: " + density.GetEventDensity(samplePoint, currentTime));
                    	Alpha1 = 1;
                    	Beta1 = 0;
                    	
                    	if(Node_on)
                    	{
                    		if(this.boostDuration > this.iteration)
                        	{
                    			if(this.boosttype ==0 || this. boosttype==3)
                    			{
                    				Alpha1 = 1;
                    				Beta1 = 0;
                    			}
                        	
                    			if(this.boosttype == 2 || this.boosttype ==5) 
                    				// boostytype 2 is concaveboost 1/p^2;
                    			{
			                		//Alpha1 = neighborEffects;
			                		//Alpha1 = AllnodesEffects(samplePoint);
			                		//Alpha1 = 100*Math.pow(neighborEffects, 2);
			                		//Alpha1 = Math.pow(verticalSamplePoint, 1)+Math.pow(horizontalSamplePoint, 1);
			                		//Alpha1 = Math.exp(neighborEffects)
			                		//Alpha1 = Math.pow(dist,1);
			                		//Alpha1 = 1-NeighborEffectsPoly(samplePoint); // gather
			                		//Alpha1 = 1-NeighborEffectsSum(samplePoint);  // gather a bit
			                		//Alpha1 = 1;
			                		//Alpha1 = 0;
			                		double p=1-neighborEffects*(1-SensingModelFunction(dist));
			                		Alpha1 = PowerGain/Math.pow(p,Power);
			                		//System.out.println("Power lamda "+Power);
			                		Beta1 = 0;
			                		//Beta1 = AllnodesEffects(samplePoint);
			                		//Beta1 = -0.005;
			                		//Beta1 = 1;
			                		//double C = -5;
			                		//Beta1 = neighborEffects * (-2*C*dist);
			                		//Beta1 = 100
			                		//Beta1 = neighborEffects;
                    		    }
                    			if(this.boosttype == 4 || this.boosttype == 6)
                    			{
                    				Alpha1 = PhiGain*Math.pow(neighborEffects, PhiPower);
                    				//Alpha1 = 1;
                    				Beta1 = 0;
                    			}
                    				

                        	}
                    	}
                    	
//                    
//                    
                    	dFdx += (Alpha1 * w1 + Beta1) * ( horizontalSamplePoint - position.x ) / dist * delta * delta;
                    	dFdy += (Alpha1 * w1 + Beta1) * ( verticalSamplePoint - position.y) / dist * delta * delta;
                  
                }
            }
        }

       // System.out.println("id: "+ id+ " dFdx: "+dFdx+" dFdy "+dFdy);
       //long midTime = myTimer.currentTimeMillis();  //for performance tuning

        dFdx2 = 0;
        dFdy2 = 0;
        

        ArrayList<point2> activeReflexVertices = new ArrayList<point2>();

        //Assuming the boundary is convex

        for (int i2 = 0; i2 < obstacles.size(); i2++)
        {
            obstacles.get(i2).GetActiveReflexVertices(position, activeReflexVertices); //must clear activeReflexVertices before calling this again

            for (int i3 = 0; i3 < activeReflexVertices.size(); i3++)
            {
                double D = point2.Dist(activeReflexVertices.get(i3), position); 
                if (D < this.sensingCutoffRange && isPointInFOV(activeReflexVertices.get(i3))) 
                {
                    boolean ARVBlocked = false; //test if an ARV is blocked by other FG
                    if (!(boundary.LineOfSight(position, activeReflexVertices.get(i3))))
                    {
                        ARVBlocked = true;
                    }
                    else
                    {
                        for (int i4 = 0; i4 < obstacles.size(); i4++)
                        {
                            if (i2 != i4)
                            {
                                if (!(obstacles.get(i4).LineOfSight(position, activeReflexVertices.get(i3))))
                                {
                                    ARVBlocked = true;
                                    break;
                                }
                            }
                        }
                    }

                    if (!ARVBlocked)
                    {
                        //SYNC
                        point2 impactPoint = new point2( -1, -1); //it should be filled with useful values before used by algorithm
                        boundary.GetImpactPoint(activeReflexVertices.get(i3), point2.ExtendToInfinite(position, activeReflexVertices.get(i3)),
                                                impactPoint, true);

                        for (int i5 = 0; i5 < obstacles.size(); i5++)
                        {
                            if (i5 != i2)
                            {
                                obstacles.get(i5).GetImpactPoint(activeReflexVertices.get(i3),
                                        point2.ExtendToInfinite(position, activeReflexVertices.get(i3)), impactPoint, false);
                            }
                        }
                        double d = point2.Dist(impactPoint, activeReflexVertices.get(i3));
                        //System.out.println("anchor:"+activeReflexVertices.get(i3).x+","+activeReflexVertices.get(i3).y);

                        double SinTheta = Math.abs(position.y - activeReflexVertices.get(i3).y) / D;
                        double CosTheta = Math.abs(position.x - activeReflexVertices.get(i3).x) / D;

                        point2 pointingOutsideNorm;
                        double sign = 1;
                        if (point2.dot(point2.minus(position, activeReflexVertices.get(i3)).Norm1(),
                                       (point2.minus(obstacles.get(i2).interiorPoint, activeReflexVertices.get(i3)))) > 0)
                        {
                            pointingOutsideNorm = point2.minus(position, activeReflexVertices.get(i3)).Norm2();
                        }
                        else
                        {
                            pointingOutsideNorm = point2.minus(position, activeReflexVertices.get(i3)).Norm1();
                        }
                        if (pointingOutsideNorm.x < 0)
                        {
                            sign = -1;
                        }

                        //Line numerical integration
                        double alpha1 = EventDensity.WALL_DECAY_FACTOR;
                        double integralSum = 0;
                        double delta1 = 0.2; //was 0.05 
                        double integrationUpperBound = d; 
                     
                      
                        
                        if ((D + d) > this.sensingCutoffRange)
                        {
                            integrationUpperBound = this.sensingCutoffRange - D;
                        }

                        point2 integralPoint = new point2();
                        for (double r = delta1 / 2; r <= integrationUpperBound; r += delta1) 
                        {
                            //point2 integralPoint = (impactPoint-activeReflexVertices.get(i3))*r/d+activeReflexVertices.get(i3);
                            // integralPoint = point2.plus(point2.divide(point2.product(point2.minus(impactPoint,
                            //        activeReflexVertices.get(i3)), r), d), activeReflexVertices.get(i3));
                            integralPoint.x = (impactPoint.x - activeReflexVertices.get(i3).x) * r / d + activeReflexVertices.get(i3).x;
                            integralPoint.y = (impactPoint.y - activeReflexVertices.get(i3).y) * r / d + activeReflexVertices.get(i3).y;
       
                            
                            
                            //boost low detection area
                                        
                           
//                           else // no boost
                                	neighborEffects = NeighborEffects(integralPoint);
                            		Phat0 = 1 - neighborEffects * (1 - SensingModelFunction(D + r));
                            		Phat = 1 - neighborEffects * (1 - alpha1 * SensingModelFunction(D + r));
                            		w2 =  density.GetEventDensity(integralPoint, currentTime) * (Phat0-Phat); 
                            		Alpha2 = 1;
                            		Beta2 = 0;
                            		if(Node_on)
                                	{
                                		if(this.boostDuration > this.iteration)
                                    	{
                                			if(this.boosttype ==0 || this. boosttype==3)
                                			{
                                				Alpha2 = 1;
                                				Beta2 = 0;
                                			}
                                    	
                                			if(this.boosttype == 2 || this.boosttype ==5) 
                                			{
                                				//Alpha2 = PowerGain/Math.pow(Phat0, Power);
                                				Alpha2 = 1;
                                				Beta2 = 0;
                                			}
                                			if(this.boosttype == 4 || this.boosttype == 6)
                                			{
                                				//Alpha2 = PhiGain*Math.pow(neighborEffects, PhiPower);
                                				//Alpha2 = PowerGain/Math.pow(Phat0, Power);
                                				Alpha2 = 1;
                                				Beta2 = 0;
                                			}
                                    	}
                                	}
                            		integralSum += delta1 * r * (Alpha2 * w2 + Beta2);	
                         }
                        dFdx2 += sign * SinTheta / D * integralSum;

                        if (pointingOutsideNorm.y < 0)
                        {
                            sign = -1;
                        }
                        else
                        {
                            sign = 1;
                        }
                        //dFdy2+=sign*CosTheta/D*(-d/sensingDecayFactor*SensingModelFunction(d+D)-1/sensingDecayFactor/sensingDecayFactor*(SensingModelFunction(d+D)-SensingModelFunction(D)));

                        dFdy2 += sign * CosTheta / D * integralSum;

                    }
                }
            }
            
            activeReflexVertices.clear(); //this container only holds ARV from one obstacle, so when we move to another obstacle, we need to empty it first.
            }

    

 		//System.out.println("Id "+this.id+" Before dFdx2 "+dFdx2+" dFdy2 "+dFdy2);
        
        //Find the maximum dFdx2 and boosting in that direction.

        

        	 dFdx = dFdx + dFdx2;
			 dFdy = dFdy + dFdy2;
           
//        if(this.id == 1)
//		{
//        	System.out.println("Id "+this.id+" neighbor size "+neighbors.size()+ " dFdx: "+dFdx + " dFdy: "+dFdy);
//		}
		//	 System.out.println("In total "+ "Id "+this.id+ " dFdx: "+dFdx + " dFdy: "+dFdy);
        // If we add a term ||si-sj|| in H(s), where j is in i's neighbor set,
        // the derivative will add (six-sjx)/||si-sj||. 
 
		if(this.boosttype == 3 || this.boosttype == 5 || this.boosttype == 6)
		{
	        if(Node_on)
	    	{
	    		if(this.boostDuration > this.iteration)
	        	{
	    			
	    			
	    		       for(int numofneighbor=0;numofneighbor < neighbors.size();numofneighbor++)
	    		        {
	    		    	   if(is_point_visible(neighbors.get(numofneighbor).position))
	    		        	{
	    		    		   double distofneighbor = point2.Dist(neighbors.get(numofneighbor).position  , position);
	    		    		   if(distofneighbor < minNeighborDistance)
	    		    		   { 
	    		    			   minNeighborDistance = distofneighbor;
	    		    			   minNeighborindex = numofneighbor;
	    		    		   }
	    		        	}
	    		        }
	    		       
	    		       //if(minNeighborDistance < 50)
	    		       //{
	    		       if(minNeighborindex != 100)
	    		       {
	    		    	   
	    		       
	    		        	 neighborDistanceSumx = neighborDistanceSumx + neighborGain*(position.x-neighbors.get(minNeighborindex).position.x )/Math.pow(minNeighborDistance, neighborPower);
	    		        	 neighborDistanceSumy = neighborDistanceSumy + neighborGain*(position.y-neighbors.get(minNeighborindex).position.y )/Math.pow(minNeighborDistance, neighborPower);
	    		        
	    		    		//neighborDistanceSumx = neighborDistanceSumx + 1/distofneighbor ;
	       		        	//neighborDistanceSumy = neighborDistanceSumy + 1/distofneighbor ;
	    		            //System.out.println("Id: "+this.id + " 	neighbor is "+ neighbors.get(minNeighborindex).id);   
	    		        
	    		       }	
	    		        
	    		        //System.out.println("Id: "+this.id + " neighborDistanceSumx "+neighborDistanceSumx+" neighborDistanceSumy"+neighborDistanceSumy);
	    		    
	    			dFdx = dFdx + neighborDistanceSumx;
	    			dFdy = dFdy + neighborDistanceSumy;
	        	}
	    	}
		}

		if(this.boosttype == 10)
		{
			System.out.println("Type=10");
			if(Node_on)
			{
									
			double randomx=Math.random()-Math.random();
			double randomy=Math.random()-Math.random();
			dFdx = dFdx + 10*randomx;
			dFdy = dFdy + 10*randomy;
			
			}
		}
		
	
		

		        // If we add a term ||si-sj|| in H(s), where j is in i's neighbor set,
		        // the derivative will add (six-sjx)/||si-sj||. 
    	
        this.iteration += 1;  		
        neighborDistanceSumx = 0;
        neighborDistanceSumy = 0;

        // perfect vehicle handling
        double gradientMagnitude = Math.sqrt(dFdx * dFdx + dFdy * dFdy);

        //if my gradient is small, in the future, I do not need to broadcast my position since it doesn't change much.
        if(gradientMagnitude<SMALL_GRADIENT_THRESHOLD)
        {
            isMyGradientMagSmall = true;
        }
        else
        {
            isMyGradientMagSmall = false;
        }

        //Asynchronous version, update the estimation error threshold based on the gradient magnitude
       if(!isFixedError)
       {
           this.estimationErrorThreshold = K4 * gradientMagnitude;
       }

        //Try to decrease the speed of robot when the gradient is smaller.
        if (gradientMagnitude < norminalGradientMagnitude)
        {
            speed = robotMaxSpeed * gradientMagnitude / norminalGradientMagnitude;
        }
        else
        {
            speed = robotMaxSpeed;
        }

        heading = Math.atan2(dFdy, dFdx); //final robot moving direction implementation
        if (heading == 0) //TODO Why heading will be zero?
        {
            speed = 0;
        }

 
        //end vehicle handling
    }
    
    synchronized double LocalObjInfo()
    {
        //Evaluate the local objective function based on the current robot positions
    	// We aim to find useful information for boosting
        double localobjective = 0;
        int coverageMapRow=0;
        int coverageMapColumn=0;

        point2 samplePoint = new point2(-1,-1);
        
        double alpha;
        try
        {
            for (double horizontalSamplePoint = 0.01; horizontalSamplePoint <= boundary.largestX;
                                                horizontalSamplePoint += sim.objEvalIncrement)
            {
                for (double verticalSamplePoint = 0.01; verticalSamplePoint <= boundary.largestY; verticalSamplePoint += sim.objEvalIncrement)
                {
                    
                    samplePoint.x = horizontalSamplePoint;
                    samplePoint.y = verticalSamplePoint;
                  

                
                    if (point2.Dist(this.position, samplePoint) < this.sensingCutoffRange) //SYNC
                    {
                        if (boundary.LineOfSight(this.position, samplePoint))
                        {
                            alpha = 1; //sensing ability discount factor
                          
                            if(!this.is_point_visible(samplePoint))
                            {
                           	 alpha = EventDensity.WALL_DECAY_FACTOR;
                            }                                   
                            
                            if(alpha>0)
                            {
                            	localobjective = localobjective + density.GetEventDensity(samplePoint) * NeighborEffects(samplePoint)* this.SensingModelFunction(point2.Dist(samplePoint, this.position));
                            }
                        }
                    }
                }
                   
            }    

                   
        }
        catch(ArrayIndexOutOfBoundsException e)
        {
       	 e.printStackTrace();
            return -1;
        }
        return localobjective * sim.objEvalIncrement * sim.objEvalIncrement;
    }

    double SensingModelFunction(double distance) //exponentially decreasing with the distance
    {
        //Make sure it is between 0 to 1
    	//if (this.id ==0)
         return Math.exp( -sensingDecayFactor * distance);
    	//else
        //return 1-0.01*distance;
    	//	return Math.exp(-sensingDecayFactor * 5 * distance);
    	//return 0.85;

    }
    double SensingModelFunctionBoost(double distance,double iteration) //exponentially decreasing with the distance
    {
    	
    	double C=-10*Math.exp(-0.4*iteration);
    	double sensingModelFunctionBoost;
        //Make sure it is between 0 to 1
    	sensingModelFunctionBoost=Math.exp( -sensingDecayFactor * distance) + C*Math.sqrt(distance);
    	//sensingModelFunctionBoost=Math.exp( -sensingDecayFactor * distance)*C;
  //      System.out.println("The result is"+sensingModelFunctionBoost);
        return sensingModelFunctionBoost;
        //return 1-0.005*distance;

    }
    double SensingModelFunctionBoost(double distance) //exponentially decreasing with the distance
    {
    	
    	double C=-1*Math.exp(-1);
    	double sensingModelFunctionBoost;
        //Make sure it is between 0 to 1
    	sensingModelFunctionBoost=Math.exp( -sensingDecayFactor * distance) + C*Math.sqrt(distance);
    	//sensingModelFunctionBoost=Math.exp( -sensingDecayFactor * distance)*C;
  //      System.out.println("The result is"+sensingModelFunctionBoost);
        return sensingModelFunctionBoost;
        //return 1-0.005*distance;
    	//return 0.85;
    }
    
    double SensingModelDerivative(double distance)
    {
    	//if (this.id ==0)
    		return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance);
    	//else
    	//return -0.01;
        //return 1/(distance+1) ;
    	//	return -5*sensingDecayFactor * Math.exp( -sensingDecayFactor * distance);

    }
    
    double SensingModelDerivativeBoost(double distance,double iteration)
    {
         double C=-10*Math.exp(-0.4*iteration);
        
    	// double C=0;
    	return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance)+2*C*distance;
    	//return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance)*2*C;
        //return -0.005;
        //return 1/(distance+1) ;
    }
    double SensingModelDerivativeBoost(double distance)
    {
         double C=-1*Math.exp(-0.4);
        
    	// double C=0;
    	return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance)+2*C*distance;
    	//return -sensingDecayFactor * Math.exp( -sensingDecayFactor * distance)*2*C;
        //return -0.005;
        //return 1/(distance+1) ;
    }
    // If there is no neighbor boost, the NeighborEffect is normal;
    synchronized double NeighborEffects(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	
        	neighborEffect *=  
        		(1 - alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position)));
        }

        return neighborEffect; //missing probability by neighbors
    }
    
    synchronized double NeighborEffectsSum(point2 samplePoint)
    {
        double neighborEffectSum = 0;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	
        	neighborEffectSum = neighborEffectSum + alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position));
        }
        return neighborEffectSum; //missing probability by neighbors
    }
    
    synchronized double AllnodesEffects(point2 samplePoint)
    {
        double allnodesEffect = 1;

        for (int i1 = 0; i1<sim.realTimeRobotNumber  ; i1++)
        {
        	double alpha = 1;
            
        	if(this.id == i1)
        	{
        		alpha = 0;
        	}
        	allnodesEffect *=  
        		(1 - alpha * sim.robotList.get(i1).SensingModelFunction(point2.Dist(samplePoint, sim.robotList.get(i1).position)));
        }

        return allnodesEffect; //missing probability by neighbors
    }
    
    synchronized double NeighborEffectsPoly(point2 samplePoint)
    {
        double neighborEffectPoly = 0;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	
        	neighborEffectPoly = neighborEffectPoly + alpha * Math.pow(neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position)), 2);
        }
        return neighborEffectPoly; //missing probability by neighbors
    }
    synchronized double NeighborEffects(point2 samplePoint, double firstTimeOn)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	 double b=Math.exp(-0.15*firstTimeOn);
        	
        	neighborEffect *=  
        		(1 - ((1-b)*alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position))+b));
 //       	double p_j= neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position));
        	//p_j=3*Math.pow(p_j, 2)-2*Math.pow(p_j, 3);
//        	neighborEffect *=  
//           		(1-alpha*(1 - Math.pow(1-p_j, 1+20/firstTimeOn)));
//        	neighborEffect *=  
//           		(1-alpha*(1 - Math.pow(1-p_j, 3)));
        }

        return neighborEffect; //missing probability by neighbors
    }
    
    synchronized double NeighborEffectsforself1(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	 double b=Math.exp(-0.8);
        	
        	neighborEffect *=  
        		(1 - ((1-b)*alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position))+b));

        }

        return neighborEffect; //missing probability by neighbors
    }
    
    synchronized double NeighborEffectsforselfboost2(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	// double b=Math.exp(-0.15*firstTimeOn);
        	
//        	neighborEffect *=  
//        		(1 - ((1-b)*alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position))+b));
        	double p_j= neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position));
        	//p_j=3*Math.pow(p_j, 2)-2*Math.pow(p_j, 3);
//        	neighborEffect *=  
//           		(1-alpha*(1 - Math.pow(1-p_j, 1+20/firstTimeOn)));
        	neighborEffect *=  
           		(1-alpha*(1 - Math.pow(1-p_j, 3)));
        }

        return neighborEffect; //missing probability by neighbors
    }
  
    synchronized double NeighborEffects2(point2 samplePoint)
    {
        double neighborEffect = 1;

        for (int i1 = 0; i1 < neighbors.size(); i1++)
        {
        	double alpha = 1;
        	
        	//TODO should use the version of is_point_visible with estimated Position and sensor_heading        	
        	if(!neighbors.get(i1).is_point_visible(samplePoint))
        	{
        		alpha = EventDensity.WALL_DECAY_FACTOR;
        	}
        	// double b=Math.exp(-0.15*firstTimeOn);
        	
//        	neighborEffect *=  
//        		(1 - ((1-b)*alpha * neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position))+b));
        	double p_j= neighbors.get(i1).SensingModelFunction(point2.Dist(samplePoint, neighbors.get(i1).position));
        	//p_j=3*Math.pow(p_j, 2)-2*Math.pow(p_j, 3);
//        	neighborEffect *=  
//           		(1-alpha*(1 - Math.pow(1-p_j, 1+20/firstTimeOn)));
        	neighborEffect *=  
           		(1-alpha*(1 - Math.pow(1-p_j, 2)));
        }
       

        return neighborEffect; //missing probability by neighbors
    }

    //calculates data collection reward
    public double monitoringModelFunction(point2 eventLocation)
    {
        return Math.exp(-dataCollectionDecay * point2.Dist(eventLocation, this.position));
    }

    public double monitoringModelDerivative(Event e)
   {
       double currectMonitoringPercentage = 0;

       for(int i = 0; i < sim.realTimeRobotNumber; i++)
       {
           currectMonitoringPercentage += sim.robotList.get(i).monitoringModelFunction(e.eventLocation);
       }

      e.monitorPercentage = currectMonitoringPercentage;

      if(currectMonitoringPercentage>=1)
      {
          return 0;
      }
      else
      {
          return -dataCollectionDecay * Math.exp( -dataCollectionDecay * point2.Dist(e.eventLocation, this.position));
      }
   }

    public void clearTrajectoryHistory()
    {
        trajectoryHistoryIndex = 0;
    }
    
    public void move_to_robot(Robot seeking_target, double range)
    {
    	this.is_seeker_mode = true;
    	this.seeker_target = seeking_target;
    	this.seeking_hit_range = range;    
    }
    
    public void set_FOV_edge_points()
    {
    	this.FOV_edge_point_left.set(position.x+sensingCutoffRange*Math.cos(FOV_edge_heading_left), position.y+sensingCutoffRange*Math.sin(FOV_edge_heading_left));
    	this.FOV_edge_point_right.set(position.x+sensingCutoffRange*Math.cos(FOV_edge_heading_right), position.y+sensingCutoffRange*Math.sin(FOV_edge_heading_right));
    }
    
//    public void shift_sensor_heading(double dFdh)  //for smoothly adjust sensor heading
//    {
//    	double max_sensor_heading_turning_speed = 0.02; //was 0.01
//    	if(dFdh>0)
//    	{
//    		this.sensor_heading -= Math.min(max_sensor_heading_turning_speed,dFdh/100);
//    	}
//    	else if(dFdh<0)
//    	{
//    		this.sensor_heading += Math.min(max_sensor_heading_turning_speed,-dFdh/100);
//    	}
//    	
//    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
//    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
//    	set_FOV_edge_points();
//    }
    
    public void modify_sensor_heading(double delta)  //for quickly adjust sensor heading
    { 
    	this.sensor_heading += delta;
 
    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    	set_FOV_edge_points();
    }
    
    public void SetSensorHeading(double newHeading)  //for quickly adjust sensor heading
    { 
    	newHeading = newHeading%(2*Math.PI);
    	this.sensor_heading = newHeading;
 
    	FOV_edge_heading_left = sensor_heading-FOV_degree/2;
    	FOV_edge_heading_right = sensor_heading+FOV_degree/2;;
    	set_FOV_edge_points();
    }
    
    public void FastSearchSensorHeading()
    {
    	double originalSensorHeading = this.sensor_heading;
    	double bestSensorHeading = this.sensor_heading;
    	double bestObjValue = this.sim.EvaluateObj();
    	
    	for(int i = 1; i<4; i++)
    	{
	    	this.SetSensorHeading(originalSensorHeading+i*Math.PI/2);
	    	double newObjValue = this.sim.EvaluateObj();
	    	if (newObjValue>bestObjValue)
	    	{
	    		bestSensorHeading = originalSensorHeading+i*Math.PI/2;
	    		bestObjValue=newObjValue;	
	    	}
    	}
    	this.SetSensorHeading(bestSensorHeading);
    }
    //TODO: implement these
//    public boolean equals(Robot r)
//    {
//    	
//    }
//    
//    public int hashCode()
//    {
//    	int hash 9;
//    	hash = (31*hash)+id;
 //   	hash = (31*hash)+(null==position?0:position.hashCode());
//    	return hash;
//    }
    

}
