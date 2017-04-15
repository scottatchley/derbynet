package org.jeffpiazza.derby.devices;

import java.util.ArrayList;
import jssc.*;
import org.jeffpiazza.derby.Message;
import org.jeffpiazza.derby.serialport.SerialPortWrapper;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

// This class supports the "Derbywizard K3" device, http://www.microwizard.com/k3page.html
public class K3Device extends TimerDeviceTypical {

  public K3Device(SerialPortWrapper portWrapper) {
    super(portWrapper);

    // Once started, we expect a race result within 10 seconds
    rsm.setMaxRunningTimeLimit(10000);
  }

  public static String toHumanString() {
    return "Derbywizard K3";
  }

  private int laneCount = 0;
  // These get initialized upon the start of each new race.
  private int nresults = 0;
  private ArrayList<Message.LaneResult> results;

  // RM => '<lanes> 000000 0 0 [01]' crlf '*' crlf
  private static final String READ_MODE = "RM";
  // RA => '*' crlf '<results>' crlf
  private static final String RESET_COMMAND = "RA";
  // M[ABCDEF] => '*' crlf
  private static final String LANE_MASK = "M";
  // MG => 'AC' crlf
  private static final String CLEAR_LANE_MASK = "MG";
  // LR => '*' crlf
  private static final String RESET_LASER_GATE = "LR";

  // Response to a "RG" is either "1" (up, or closed) or "0" (down, or open)
  private static final String READ_GATE = "RG";

  private static final String FORCE_RACE_RESULTS = "RX";

  public boolean probe() throws SerialPortException {
    if (!portWrapper.setPortParams(SerialPort.BAUDRATE_9600,
                                   SerialPort.DATABITS_8,
                                   SerialPort.STOPBITS_1,
                                   SerialPort.PARITY_NONE,
                                   /* rts */ true,
                                   /* dtr */ true)) {
      return false;
    }

    // Ignore the copyright and version
    portWrapper.drain(500, 3);
    // Call READ MODE to get the lane count
    portWrapper.writeAndWaitForResponse(READ_MODE, 500);

    long deadline = System.currentTimeMillis() + 2000;
    String s;
    while ((s = portWrapper.next(deadline)) != null) {
        Matcher m = readLaneCountPattern.matcher(s);
        if (m.find()) {
          laneCount = Integer.parseInt(m.group(1));
	  // Ignore '*' on the next line
	  portWrapper.drain();
        }

        setUp();
        return true;
    }

    return false;
  }

  private static final Pattern readModePattern = Pattern.compile(
	"^\\d\\s\\d\\d\\d\\d\\d\\d\\s\\d\\s\\d\\s\\d");
  private static final Pattern readLaneCountPattern = Pattern.compile(
	"(^\\d)(.*)");
  //A=1.234! B=2.345" C=3.456# D=4.567$ E=5.678% F=6.789&
  private static final Pattern readResultsPattern = Pattern.compile(
	"A=(\\d\\.\\d+). B=(\\d\\.\\d+). C=(\\d\\.\\d+). D=(\\d\\.\\d+). E=(\\d\\.\\d+). F=(\\d\\.\\d+).");
  private static final Pattern readyNLanesPattern = Pattern.compile(
      "^READY\\s*(\\d+)\\s+LANES");
  private static final Pattern singleLanePattern = Pattern.compile(
      "^\\s*(\\d)\\s+(\\d\\.\\d+)(\\s.*|)");

  protected void setUp() {
    portWrapper.registerDetector(new SerialPortWrapper.Detector() {
      @Override
      public String apply(String line) throws SerialPortException {
	Matcher m = readResultsPattern.matcher(line);

        for (int lane = 1; lane < laneCount; lane++) {
          String time = m.group(lane);
	  nresults++;
          TimerDeviceUtils.addOneLaneResult(lane, time, nresults, results);
	}
        if (results != null) {
          raceFinished((Message.LaneResult[]) results.toArray(
              new Message.LaneResult[results.size()]));
          results = null;
          nresults = 0;
          return "";
        }

        } else {
          Matcher m = readyNLanesPattern.matcher(line);
          if (m.find()) {
            int nlanes = Integer.parseInt(m.group(1));
            // If any lanes have been masked, not sure what READY n LANES
            // will report, so only update a larger laneCount.
            if (nlanes > laneCount) {
              laneCount = nlanes;
            }
            if (!getGateIsClosed()) {
              setGateIsClosed(true);
              rsm.onEvent(RacingStateMachine.Event.GATE_CLOSED,
                          K3Device.this);
            }
            return "";
          }
          return line;
        }
      }
    });
  }

  // TODO synchronized?
  public synchronized void prepareHeat(int roundid, int heat, int lanemask)
      throws SerialPortException {
    RacingStateMachine.State state = rsm.state(this);
    // TODO This isn't necessary if the server won't send a redundant heat-ready
    // No need to bother doing anything if we're already prepared for this heat.
    if (this.roundid == roundid && this.heat == heat
        && (state == RacingStateMachine.State.MARK
            || state == RacingStateMachine.State.SET)) {
      portWrapper.logWriter().traceInternal("Ignoring redundant prepareHeat()");  // TODO
      return;
    }

    prepare(roundid, heat);
    nresults = 0;
    results = new ArrayList<Message.LaneResult>();

    portWrapper.writeAndDrainResponse(CLEAR_LANE_MASK);

    StringBuilder sb = new StringBuilder("Heat prepared: ");
    for (int lane = 0; lane < laneCount; ++lane) {
      if ((lanemask & (1 << lane)) != 0) {
        sb.append(lane + 1);
      } else {
        sb.append("-");
        // Response is "MASKING LANE <n>"
        portWrapper.writeAndDrainResponse(
            LANE_MASK + (char) ('A' + lane), 1, 500);
      }
    }
    // Reset the laser gate
    portWrapper.writeAndDrainResponse(RESET_LASER_GATE, 1, 500);
    portWrapper.logWriter().serialPortLogInternal(sb.toString());
    rsm.onEvent(RacingStateMachine.Event.PREPARE_HEAT_RECEIVED, this);
  }

  // Interrogates the starting gate's state.
  @Override
  protected synchronized boolean interrogateGateIsClosed()
      throws NoResponseException, SerialPortException, LostConnectionException {
    portWrapper.write(READ_GATE);
    long deadline = System.currentTimeMillis() + 1000;
    String s;
    while ((s = portWrapper.next(deadline)) != null) {
      if (s.trim().equals("1")) {
        return true;
      } else if (s.trim().equals("0")) {
        return false;
      } else {
        portWrapper.logWriter().serialPortLogInternal(
            "Unrecognized response: '" + s + "'");
      }
    }

    throw new NoResponseException();
  }

  public int getNumberOfLanes() throws SerialPortException {
    return laneCount;
  }

  @Override
  public void onTransition(RacingStateMachine.State oldState,
                           RacingStateMachine.State newState)
      throws SerialPortException {
    if (newState == RacingStateMachine.State.RESULTS_OVERDUE) {
      // Force results upon entering RESULTS_OVERDUE.  After another second
      // (in whileInState), give up and revert to idle.
      portWrapper.write(FORCE_RACE_RESULTS);
      logOverdueResults();
    }
  }

  protected void whileInState(RacingStateMachine.State state)
      throws SerialPortException, LostConnectionException {
    if (state == RacingStateMachine.State.RESULTS_OVERDUE) {

      // A reasonably common scenario is this: if the gate opens accidentally
      // after the PREPARE_HEAT, the timer starts but there are no cars to
      // trigger a result.
      //
      // updateGateIsClosed() may throw a LostConnectionException if the
      // timer has become unresponsive; otherwise, we'll deal with an
      // unexpected gate closure (which has no real effect).
      if (updateGateIsClosed()) {
        // It can certainly happen that the gate gets closed while the race
        // is running.
        rsm.onEvent(RacingStateMachine.Event.GATE_CLOSED, this);
      }

      if (rsm.millisInCurrentState() > 1000) {
        // TODO invokeMalfunctionCallback(false,
        //                                "No result received from last heat.");
        // We'd like to alert the operator to intervene manually, but
        // as currently implemented, a malfunction(false) message would require
        // unplugging/replugging the timer to reset: too invasive.
        portWrapper.logWriter().serialPortLogInternal(
            "No result from timer for the running race; giving up.");
        // This forces the state machine back to IDLE.
        rsm.onEvent(RacingStateMachine.Event.RESULTS_RECEIVED, this);
      }
    }
  }
}
