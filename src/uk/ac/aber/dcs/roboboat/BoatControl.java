package uk.ac.aber.dcs.roboboat;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.location.Criteria;
import android.location.Location;
import android.location.LocationListener;
import android.location.LocationManager;
import android.os.Bundle;
import android.os.Handler;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;
import ioio.lib.api.AnalogInput;
import ioio.lib.api.DigitalInput;
import ioio.lib.api.DigitalOutput;
import ioio.lib.api.DigitalOutput.Spec;
import ioio.lib.api.PulseInput;
import ioio.lib.api.PulseInput.PulseMode;
import ioio.lib.api.PwmOutput;
import ioio.lib.api.exception.ConnectionLostException;
import ioio.lib.util.BaseIOIOLooper;
import ioio.lib.util.IOIOLooper;
import ioio.lib.util.android.IOIOActivity;

public class BoatControl extends IOIOActivity implements SensorEventListener,  LocationListener {

	private TextView textView_;
	private TextView headingText_;
	private SeekBar sailSeekBar_;
	private SeekBar rudderSeekBar_;
	public Context context;
	private LocationManager gps;
	private String gpsProvider;
	private BoatState state = new BoatState();
	private float[] gravity = new float[3];
	private float[] geomag = new float[3];

	//public PhoneSensors phoneSensors;

	SensorManager m_sensorManager;
	
	private void registerListeners() {
		m_sensorManager.registerListener(this,
				m_sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
				SensorManager.SENSOR_DELAY_FASTEST);
		m_sensorManager.registerListener(this,
				m_sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_FASTEST);
		m_sensorManager.registerListener(this,
				m_sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
				SensorManager.SENSOR_DELAY_FASTEST);
		
		gps = (LocationManager)this.getBaseContext().getSystemService(Context.LOCATION_SERVICE);
		Criteria criteria = new Criteria();
		criteria.setAccuracy(Criteria.ACCURACY_FINE);
		gpsProvider = gps.getBestProvider(criteria, true);
		gps.requestLocationUpdates(gpsProvider, 1000, 0, this);
		Location location = gps.getLastKnownLocation(gpsProvider);
		if(location!=null)
		{
			double lat = location.getLatitude(); 
			state.setLat(lat);
			state.setLon(location.getLongitude());
			headingText_.setText((new Float(state.getLat()).toString()) + (new Float(state.getLon()).toString()));
		}
	}

	private void unregisterListeners() {
		m_sensorManager.unregisterListener(this);
	}
	
	@Override
	public void onLocationChanged(Location location) {
		state.setLat(location.getLatitude());
		state.setLon(location.getLongitude());
		headingText_.setText((new Float(state.getLat()).toString()) + (new Float(state.getLon()).toString()));
	}

	@Override
	public void onDestroy() {
		unregisterListeners();
		super.onDestroy();
	}

	@Override
	public void onPause() {
		unregisterListeners();
		super.onPause();
	}

	@Override
	public void onResume() {
		registerListeners();
		super.onResume();
	}

	@Override
	public void onAccuracyChanged(Sensor sensor, int accuracy) {
	}

	@Override
	public void onSensorChanged(SensorEvent event) {
		float[] inR = new float[16];
		float[] orientVals = new float[3];
		float[] I = new float[16];
		//float[] orientVals = new float[3];
		float[] apr = new float[3];
		
		if (event.sensor.getType() == Sensor.TYPE_MAGNETIC_FIELD) {
			geomag = event.values.clone();
		}
		else if(event.sensor.getType() == Sensor.TYPE_ACCELEROMETER ) {
			
			gravity = event.values.clone();
		}
		
		if(gravity != null && geomag != null) {
			boolean success =
				SensorManager.getRotationMatrix(inR, I, gravity, geomag);
			if(success) {
				SensorManager.getOrientation(inR, orientVals);
				state.setHeading((int)Math.toDegrees((double)orientVals[0]));
				
	

			}
		}
	}


	
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);

		textView_ = (TextView) findViewById(R.id.console);
		headingText_ = (TextView) findViewById(R.id.headingText);
		rudderSeekBar_ = (SeekBar) findViewById(R.id.rudderDirection);
		sailSeekBar_ = (SeekBar) findViewById(R.id.sailDirection);

		m_sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
		registerListeners();

		enableUi(false);
	}

	class Looper extends BaseIOIOLooper {

		public PwmOutput SERVO_RUDDER;
		public PwmOutput SERVO_SAIL;
		public static final int WIND_SENSOR = 44;

		/* Wind sensor PWM ranges (relative to sensor not world) */
		public static final double ZERO_DEGREES = 3.105000068899244E-4;

		private HTTPServer httpServer;
		private DigitalOutput led;
		private boolean led_ = true;
		private PwmOutput[] servos;
		private int angle = 0;
		private double windDirection = 0;
		private double windForward;
		private int servo_sail_angle;
		private int servo_rudder_angle;
		private AnalogInput windPulse;
		private int[] angles;
		private WaypointManager wm = new WaypointManager(state);
		int rudderPos = 1000;
		int sailPos = 1000;
		int PORT_TACK, STBD_TACK;
		float running_err;
		float pgain = (float) 0.1;
		float igain = (float) 0.01;

		private double windCalibrated = 0;
		Handler handler = new Handler();

		// calculates difference between two headings taking wrap around into
		// account
		int get_hdg_diff(int heading1, int heading2) {
			int result;

			result = heading1 - heading2;

			if (result < -180) {
				result = 360 + result;
				return result;
			}

			if (result > 180) {
				result = 0 - (360 - result);
			}

			return result;
		}

		/*
		 * works out if we should be tacking or not returns a new heading that
		 * reflects this
		 */
		int check_tacking(int relwind, int heading, int des_hdg) {
			int truewind, tempwpthdg, temptruewind;
			int HOW_CLOSE = 45;

			truewind = relwind + heading; // Calculate true wind direction
			if (truewind > 360)
				truewind -= 360;
			// handle tacking
			// handle when differenve over 180
			if ((Math.abs(truewind - des_hdg)) > 180) {
				if ((360 - (Math.abs(truewind - des_hdg))) < HOW_CLOSE) {
					state.setSailable(false);
				} else {
					state.setSailable(true);
				}
			}
			// when difference less than 180
			else if (Math.abs(truewind - des_hdg) < HOW_CLOSE) // Only try to
																// sail to
																// within 55
																// degrees of
																// the wind
			{
				state.setSailable(false);
			} else {
				state.setSailable(true);
				PORT_TACK = 0;
				STBD_TACK = 0;
			}

			// why not just if !sailable??
			if ((state.isSailable() == false) && (PORT_TACK == 0)
					&& (STBD_TACK == 0)) // If we can't lay the course to the
											// waypoint then...
			{
				temptruewind = truewind;
				tempwpthdg = des_hdg;
				if (des_hdg < HOW_CLOSE) {
					tempwpthdg += 180;
					temptruewind += 180;
					if (temptruewind > 360) {
						temptruewind -= 360;
					}
				}
				if (des_hdg > (360 - HOW_CLOSE)) {
					tempwpthdg -= 180;
					temptruewind -= 180;
					if (temptruewind < 0) {
						temptruewind += 360;
					}
				}
				if (tempwpthdg > temptruewind) {
					// des_hdg = truewind + 55; // Sail 55 degrees off the wind
					// on the relevant tack
					PORT_TACK = 1; // Set flag to stop boat "short tacking" to
									// waypoint
				} else {
					// des_hdg = truewind - 55; // to get as close as possible
					STBD_TACK = 1; // Set flag to stop boat "short tacking" to
									// waypoint
				}
				// printf("Tgt hdg adj to: %f\n",target_hdg);

			} // otherwise just sail directly to the waypoint

			if (state.isSailable() == false) {
				// Keep boat hard on wind on same tack until we can lay course
				// for waypoint (enforce single tack)
				if (PORT_TACK == 1) {
					des_hdg = truewind + HOW_CLOSE; // Sail 55 degrees off the
													// wind on port tack
				}

				if (STBD_TACK == 1) {
					des_hdg = truewind - HOW_CLOSE; // Sail 55 degrees off the
													// wind on stbd tack
				}
			}

			if (des_hdg > 359) {
				des_hdg -= 360;
			}

			if (des_hdg < 0) {
				des_hdg += 360;
			}

			return des_hdg;
		}

		@Override
		public void setup() throws ConnectionLostException {
			try {

				log("IOIO Connected,  configuring");
				log("Ensure wind sensor is facing forward");
				Thread.sleep(500);
				led = ioio_.openDigitalOutput(0, false);

				SERVO_SAIL = ioio_.openPwmOutput(6, 50);
				SERVO_RUDDER = ioio_.openPwmOutput(7, 50);

				windPulse = ioio_.openAnalogInput(WIND_SENSOR);
				windForward = readWindSensor();
				// windPulse = ioio_.openDigitalInput(WIND_SENSOR);
				// windCalibrated = calibrateWindSensor();

				log("Sail Servo output on pin 6");
				log("Rudder Servo output on pin 7");
				log("Wind Sensor input on pin " + WIND_SENSOR);
				log("-----------------------------");
				log("Thunderbirds are go!");
			} catch (Exception e) {
				log("IOIO Not configured correctly!");
				log("Exception: " + e.getMessage());
			}
			//phoneSensors = new PhoneSensors(context);

			/* httpServer = new HTTPServer(); */

		}

		@Override
		public void loop() throws ConnectionLostException, InterruptedException {
			int new_sail_pos;
			int new_rudder_pos;
			led_ = !led_;
			led.write(led_);

			// SERVO_SAIL.setPulseWidth(servo_sail_angle);
			// SERVO_RUDDER.setPulseWidth(servo_rudder_angle);
			// windDirection = windPulse.getDuration();
			Thread.sleep(1000);

			windDirection = (readWindSensor() - windForward) * 360;
			if (windDirection < 0) {
				windDirection = windDirection * -1;
			}

			// convert windDirection here
			state.setWindDir(((int) windDirection) % 360);
			log("wind = " + new Integer((int)windDirection).toString() + " hdg = " + state.getHeading() + "loc =" + state.getLat() + "," + state.getLon());
			/*
			 * wm.waypointCheck(phoneSensors); og("Wind dir = " + windDirection
			 * + "hdg=" + phoneSensors.getAzimuth());
			 */
			// + "lat = " + context.getSensors().getLat() + "lon=" +
			// context.getSensors().getLon() +

			// ********* rudder pi controller
			int target_hdg = check_tacking(state.getRelwind(),
					state.getHeading(), state.getDesiredHeading());

			float error = (float) get_hdg_diff(state.getHeading(), target_hdg);
			// int force_jibe(int des_hdg,int heading,int relwind)

			// error = (float)
			// force_jibe((int)target_hdg,(int)hdg,(int)relwind);
			if (error > 180.0) {
				error = (float) 180.0;
			} else if (error < -180.0) {
				error = (float) -180.0;
			}

			running_err = running_err + error;
			if (Math.abs(running_err) > 4000)
				running_err = 4000; // limit integral component
			running_err = running_err * (float) 0.9;

			new_rudder_pos = (int) ((error * pgain) + (running_err * igain));

			// rudderPos=new_rudder_pos*1;

			setServoAngleRaw(SERVO_RUDDER, rudderPos);

			if (state.getRelwind() >= 0 && state.getRelwind() < 15) // in irons,
																	// help us
																	// tack?
			{
				new_sail_pos = 0;
			} else if (state.getRelwind() >= 15 && state.getRelwind() < 50) // close
																			// hauled
			{
				new_sail_pos = 1;
			} else if (state.getRelwind() >= 50 && state.getRelwind() < 80) // close
																			// reach
			{
				new_sail_pos = 2;
			} else if (state.getRelwind() >= 80 && state.getRelwind() < 120) // beam
																				// reach
			{
				new_sail_pos = 3;
			} else if (state.getRelwind() >= 120 && state.getRelwind() < 145) // broad
																				// reach
			{
				new_sail_pos = 4;
			} else if (state.getRelwind() < 215) // run
			{
				new_sail_pos = 5;
			} else if (state.getRelwind() >= 215 && state.getRelwind() < 240) // broad
																				// reach
			{
				new_sail_pos = 4;
			} else if (state.getRelwind() >= 240 && state.getRelwind() < 280) // beam
																				// reach
			{
				new_sail_pos = 3;
			} else if (state.getRelwind() >= 280 && state.getRelwind() < 310) // close
																				// hauled
			{
				new_sail_pos = 2;
			} else if (state.getRelwind() <= 345) {
				new_sail_pos = 1;
			} else {
				new_sail_pos = 0;
			}

			// sailPos = new_sail_pos * 1;

			setServoAngleRaw(SERVO_SAIL, sailPos);
			rudderPos += 50;
			sailPos += 50;
			if (rudderPos > 2000) {
				rudderPos = 1000;
			}
			if (sailPos > 2000) {
				sailPos = 1000;
			}
		}

		@Override
		public void disconnected() {
			log("IOIO Board disconnected");

		}

		@Override
		public void incompatible() {
			log("IOIO Board incompatible with API");

		}

		/**
		 * Log a message to the console on the app interface. Obviously only
		 * works if the app is visible, otherwise nothing is logged.
		 * 
		 * @param msg
		 *            Message to log
		 */
		/*
		 * public void log(final String msg) { final ControlActivity c =
		 * this.context; if (c == null) return; c.runOnUiThread(new Runnable() {
		 * 
		 * @Override public void run() { c.log(msg); } }); }
		 */

		public double getWindDirection() {
			return this.windDirection;
		}

		/**
		 * Move a servo to the specified angle in degrees. Angle is from -90
		 * (full left) to +90 (full right), with 0 as centered. <b>Does not move
		 * immediately;</b> servo will change direction during the next main
		 * loop iteration (BoatControl.loop()).
		 * 
		 * @param servo
		 *            Servo pin number
		 * @param angle
		 *            Angle (in degrees) to turn servo to
		 */
		public void setServoAngle(PwmOutput servo, int angle) {
			setServoAngleRaw(servo, angleInMicros(angle));
		}

		/**
		 * Move a servo to specified angle by giving it a new pulse width. Range
		 * is 1000 to 2000 (microseconds). <b>Also does not move
		 * immediately;</b> servo will change direction during the next main
		 * loop iteration (BoatControl.loop()).
		 * 
		 * @param servo
		 *            Servo pin number
		 * @param angle
		 *            Angle (in microseconds) to turn servo to
		 */
		public void setServoAngleRaw(PwmOutput servo, int angle) {
			// log("Servo " + servo + ": angle = " + angle + "uS");
			if (servo != null) {
				/*
				 * try { servo.setPulseWidth(angle); } catch
				 * (ConnectionLostException e) { log("Servo ConnectionLost"); }
				 */
			}
		}

		/**
		 * Convert an angle (in degrees) into a pulse width, to be used by
		 * common servos (1000 microseconds = full left, 2000 = full right, 1500
		 * = centered). <b>NOTE:</b> Clamps values in the range > -90 to < +90.
		 * 
		 * @param degrees
		 * @return
		 */
		public int angleInMicros(int degrees) {
			if (degrees < -90)
				return 1000;
			if (degrees > 90)
				return 1000;
			return (int) (((float) degrees + 90) / 0.18) + 1000;
		}

		/* This is Colin's way */
		/* Slooooow on IOIO. Only works on PIC. */
		/*
		 * public double readWindSensorRaw() { int high_count=0, low_count=0;
		 * try{ for(int i=0; i<20; i++) { while(windPulse.read() == false) {
		 * low_count++; } while(windPulse.read() == true) { high_count++; } } }
		 * catch(Exception e) { } double angle = 0.0; angle =
		 * high_count/((high_count+low_count)/360);
		 * 
		 * if(angle > 359) angle = 359; if(angle < 0) angle = 0; return angle; }
		 */

		public double readWindSensor() {
			// return (readWindSensorRaw()+windCalibrated) % 360;
			try {
				return windPulse.read();
			} catch (Exception e) {
				return 0.0;
			}
		}

		/*
		 * public double calibrateWindSensor() { return readWindSensorRaw(); }
		 */
	} // end of inner class

	@Override
	protected IOIOLooper createIOIOLooper() {
		return new Looper();
	}

	private void enableUi(final boolean enable) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				sailSeekBar_.setEnabled(enable);
				rudderSeekBar_.setEnabled(enable);
				// toggleButton_.setEnabled(enable);
			}
		});
	}

	public void log(final String msg) {

		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				textView_.setText(textView_.getText() + "\n" + msg);
				int len = textView_.getText().length();
				if (len > 200) {
					textView_.setText(textView_.getText().toString()
							.substring(len - 200, len));
				}
			}
		});

		// TODO Auto-generated method stub

	}

	@Override
	public void onProviderDisabled(String arg0) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onProviderEnabled(String provider) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public void onStatusChanged(String provider, int status, Bundle extras) {
		// TODO Auto-generated method stub
		
	}
}
