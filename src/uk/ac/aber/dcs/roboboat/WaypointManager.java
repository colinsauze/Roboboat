package uk.ac.aber.dcs.roboboat;

import java.util.Vector;

public class WaypointManager {

	Vector <Waypoint>waypointList = new Vector<Waypoint>();
	int currentWaypoint = 0;
	final static double WAYPOINT_THRESHOLD = 0.01;
	BoatState state;
	
	public WaypointManager(BoatState theState)
	{
		state = theState;
	}
	
	
	public double get_XTE()
	{
	    double dist_AB,bearing_AC,bearing_AB;
	    //calculate cross track error (XTE)
	    //make sure that there are at least 2 waypoints and that we heve reached the first one
	    if(state.getWaypointNum()>0&&state.getNumOfWaypoints()>1)
	    {
	        dist_AB=get_distance(waypointList.get(state.getWaypointNum()-1).getLat(),
	        		waypointList.get(state.getWaypointNum()-1).getLon(),state.getLat(),state.getLon());
	        //double get_course(double lat1,double lon1,double lat2,double lon2 )
	        bearing_AC=BoatState.deg2rad(get_course(waypointList.get(state.getWaypointNum()-1).getLat(),
	        		waypointList.get(state.getWaypointNum()-1).getLon(),state.getLat(),state.getLon()));
	        
	        bearing_AB=BoatState.deg2rad(get_course(waypointList.get(state.getWaypointNum()-1).getLat(),   		waypointList.get(state.getWaypointNum()-1).getLon(),
	        		waypointList.get(state.getWaypointNum()).getLat(),
	        		waypointList.get(state.getWaypointNum()).getLon()));
	        
	        //A start point, B end point, C current point
	        return(Math.asin(Math.sin(dist_AB/6367)*Math.sin(bearing_AC-bearing_AB)) * 6367);
	    }
	    return(0.0);
	}


	protected double get_distance(double lat1,double lon1,double lat2,double lon2 )
	{
	   double dlon = lon2 - lon1;
	   double dlat = lat2 - lat1;
	   double a = Math.pow(Math.sin(dlat/2),2) + Math.cos(lat1) * Math.cos(lat2) * Math.pow(Math.sin(dlon/2),2);
	   double d = 2 * Math.atan2(Math.sqrt(a),Math.sqrt(1-a));
	   //halfway between equatorial radius (6378km) and polar radius(6357km)
	   double dist = 6367 * d;
	   
	   return dist;
	}

	public double get_course(double lat1,double lon1,double lat2,double lon2 )
	{
	    double heading=Math.atan2(Math.sin(lon2-lon1)*Math.cos(lat2), Math.cos(lat1)*Math.sin(lat2)-Math.sin(lat1)*Math.cos(lat2)*Math.cos(lon2-lon1));
	    heading = heading * (180/Math.PI);
	    
	    //make headings between 0 and 360 not -180 and +180
	    if(heading<0)
	    {
		heading = heading + 360;
	    }
	    return heading;
	}

	
	public void addWaypoint(double lat,double lon)
	{
		waypointList.add(new Waypoint(lat,lon));
	}
	
	public void waypointCheck(PhoneSensors sensors)
	{
		double dist,course;
		
		state.setLat(sensors.getLat());
		state.setLon(sensors.getLon());
		state.setWaypointDist(get_distance(state.getLat(),state.getLon(),waypointList.get(currentWaypoint).getLat(),waypointList.get(currentWaypoint).getLon()));
		state.setDesiredHeading((int)get_course(state.getLat(),state.getLon(),waypointList.get(currentWaypoint).getLat(),waypointList.get(currentWaypoint).getLon()));
		state.setXte(get_XTE());
		
		if(state.getWaypointDist()<WAYPOINT_THRESHOLD)
		{
			if(state.getWaypointNum()<state.getNumOfWaypoints()) //should this be -1?, checkign we have a waypoint left
			{
				state.setWaypointNum(state.getWaypointNum()+1);
				state.setWaypointDist(get_distance(state.getLat(),state.getLon(),waypointList.get(currentWaypoint).getLat(),waypointList.get(currentWaypoint).getLon()));
				state.setDesiredHeading((int)get_course(state.getLat(),state.getLon(),waypointList.get(currentWaypoint).getLat(),waypointList.get(currentWaypoint).getLon()));
				state.setXte(get_XTE());
			}
		}
	}

	
}
