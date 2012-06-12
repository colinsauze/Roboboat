package uk.ac.aber.dcs.roboboat;

public class BoatState {

	double lat,lon;
	int heading;
	int desiredHeading;
	public int getDesiredHeading() {
		return desiredHeading;
	}

	public void setDesiredHeading(int desiredHeading) {
		this.desiredHeading = desiredHeading;
	}

	int waypointNum;
	int numOfWaypoints;
	
	double waypointDist;
	double xte;
	double speed;
	int truewind;
	int relwind;
	boolean autonomous;
	boolean sailable;
	
	final static double WIND_DECAY_RATE=250.0;
	
	double wind_avg_sin=0.0,wind_avg_cos=0.0;
	int temp_wind_dir;

	public static double rad2deg(double x) {
		return (180/Math.PI) * x;
	}
	
	public static double deg2rad(double x) {
		return (x * Math.PI/180);
	}
	
	public void setWindDir(int newRelwind)
	{
	    //int16_t wind_dir; //this is in common_vars.h
	    int i;
	    double cur_sin,cur_cos,avg_angle;

        i=newRelwind;
        i=(i+heading)%360;

        cur_sin=Math.sin(deg2rad((double)i));
        cur_cos=Math.cos(deg2rad((double)i));

        //take a running average of the sine and cosine of the wind angle
        wind_avg_sin = wind_avg_sin + ((cur_sin-wind_avg_sin) / WIND_DECAY_RATE);
        wind_avg_cos = wind_avg_cos + ((cur_cos-wind_avg_cos) / WIND_DECAY_RATE);

        avg_angle=rad2deg(Math.atan2(wind_avg_sin,wind_avg_cos));
        if(avg_angle<0.0)
        {
            avg_angle=avg_angle+360.0;
        }

        truewind=(int)avg_angle;
        
        relwind=(truewind - heading); 
        if(relwind<0)
        {
            relwind=relwind+360;
        }
    }


	public double getLat() {
		return lat;
	}


	public void setLat(double lat) {
		this.lat = lat;
	}


	public double getLon() {
		return lon;
	}


	public void setLon(double lon) {
		this.lon = lon;
	}


	public int getHeading() {
		return heading;
	}


	public void setHeading(int heading) {
		this.heading = heading;
	}


	public int getWaypointNum() {
		return waypointNum;
	}


	public void setWaypointNum(int waypointNum) {
		this.waypointNum = waypointNum;
	}


	public int getNumOfWaypoints() {
		return numOfWaypoints;
	}


	public void setNumOfWaypoints(int numOfWaypoints) {
		this.numOfWaypoints = numOfWaypoints;
	}


	public double getWaypointDist() {
		return waypointDist;
	}


	public void setWaypointDist(double waypointDist) {
		this.waypointDist = waypointDist;
	}


	public double getXte() {
		return xte;
	}


	public void setXte(double xte) {
		this.xte = xte;
	}


	public double getSpeed() {
		return speed;
	}


	public void setSpeed(double speed) {
		this.speed = speed;
	}


	public int getTruewind() {
		return truewind;
	}


	public void setTruewind(int truewind) {
		this.truewind = truewind;
	}


	public void setRelwind(int relwind) {
		this.relwind = relwind;
	}

	public int getRelwind() {
		return relwind;
	}

	public boolean isAutonomous() {
		return autonomous;
	}

	public void setAutonomous(boolean autonomous) {
		this.autonomous = autonomous;
	}

	public boolean isSailable() {
		return sailable;
	}

	public void setSailable(boolean sailable) {
		this.sailable = sailable;
	}

	
}
