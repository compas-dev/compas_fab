/* COMPAS FAB — Grasshopper component icons
   All icons authored on a 24x24 grid, ~3px padding, 1.8 monoline stroke.
   class "ac"  -> accent stroke   |  class "adot" -> accent fill
   default     -> ink stroke      |  class "dot"  -> ink fill
   The semantic SUBJECT of each icon carries the accent. */

const ICONS = {
  /* ---------------- ROBOT CELL ---------------- */
  // iso-cube (the cell) sitting on a stack of books (the library)
  cellLibrary:
    '<path d="M11 3.5 L15 6 L11 8.5 L7 6 Z"/>' +
    '<path d="M7 6 V10.5 L11 13 L15 10.5 V6 M11 8.5 V13"/>' +
    '<path class="ac" d="M4 20 H16 V17.6 H4 Z"/>' +
    '<path class="ac" d="M5.6 17.6 V15.3 H17.6 V17.6"/>' +
    '<path class="ac" d="M6.5 20 V17.6 M8 17.6 V15.3"/>',

  // cell loaded from a URDF/SRDF document
  cellUrdf:
    '<path d="M7 4 H14 L17 7 V20 H7 Z"/>' +
    '<path d="M14 4 V7 H17"/>' +
    '<path class="ac" d="M10 16.5 H14.5 M10 16.5 V12"/>' +
    '<circle class="adot" cx="10" cy="16.5" r="1.3"/>',

  // cell streamed from a ROS bridge
  cellRos:
    '<path d="M9 9 L12.5 11 L9 13 L5.5 11 Z"/>' +
    '<path d="M5.5 11 V14.6 L9 16.6 L12.5 14.6 V11 M9 13 V16.6"/>' +
    '<path class="ac" d="M15.5 11.2 a3 3 0 0 1 0 -4 M17.8 12 a6 6 0 0 0 0 -7.5"/>' +
    '<circle class="adot" cx="15" cy="6.4" r="1.2"/>',

  // add a tool (cone) to the cell
  addTool:
    '<path d="M5 7 H13 L9 16 Z"/>' +
    '<circle class="dot" cx="9" cy="16" r="1.2"/>' +
    '<path class="ac" d="M18 5 V9 M16 7 H20"/>',

  // add a rigid body (box) to the cell
  addBody:
    '<rect x="5" y="8" width="9" height="9" rx="1"/>' +
    '<path class="ac" d="M18 5 V9 M16 7 H20"/>',

  // rigid body built from a mesh (triangulated patch)
  bodyFromMesh:
    '<path d="M6 7 H18 V19 H6 Z"/>' +
    '<path d="M6 7 L13 12 L18 7 M13 12 L6 19 M13 12 L18 19"/>' +
    '<circle class="adot" cx="13" cy="12" r="1.3"/>',

  // tool (cone) built from a Rhino mesh + TCP
  toolFromMesh:
    '<path d="M5.5 7 H16.5 L11 17.5 Z"/>' +
    '<path d="M5.5 7 L11 11 L16.5 7 M11 11 L8.2 7 M11 11 L13.8 7 M11 11 V17.5"/>' +
    '<circle class="adot" cx="11" cy="17.5" r="1.3"/>',

  // tool (cone) pulled from a stack of books (the library)
  toolFromLibrary:
    '<path d="M6.5 4 H15.5 L11 12 Z"/>' +
    '<circle class="dot" cx="11" cy="12" r="1.1"/>' +
    '<path class="ac" d="M4 20 H16 V17.6 H4 Z"/>' +
    '<path class="ac" d="M5.6 17.6 V15.3 H17.6 V17.6"/>' +
    '<path class="ac" d="M6.5 20 V17.6 M8 17.6 V15.3"/>',

  // rigid body (box) pulled from a stack of books (the library)
  bodyFromLibrary:
    '<rect x="7" y="3.5" width="8" height="8" rx="1"/>' +
    '<path class="ac" d="M4 20 H16 V17.6 H4 Z"/>' +
    '<path class="ac" d="M5.6 17.6 V15.3 H17.6 V17.6"/>' +
    '<path class="ac" d="M6.5 20 V17.6 M8 17.6 V15.3"/>',

  /* ---------------- CELL STATE ---------------- */
  // default state: robot at home (articulated arm at rest)
  defaultState:
    '<path d="M6 20 H11 V18.2 H6 Z"/>' +
    '<path d="M8.5 18.2 V12.6"/>' +
    '<circle class="dot" cx="8.5" cy="12.6" r="1.4"/>' +
    '<path d="M8.5 12.6 L14 9.2"/>' +
    '<circle class="dot" cx="14" cy="9.2" r="1.3"/>' +
    '<path d="M14 9.2 L17.6 10"/>',

  // override the robot configuration in a state
  setConfig:
    '<path d="M6.5 20 H11.5 V18 H6.5 Z"/>' +
    '<path d="M9 18 V13 L13.5 10.5"/>' +
    '<circle class="dot" cx="9" cy="13" r="1.3"/>' +
    '<path class="ac" d="M14.5 16.5 A4 4 0 1 1 18 13.2"/>' +
    '<path class="ac" d="M18.4 10.6 L18.1 13.4 L15.4 12.9"/>',

  // attach a tool to the robot flange
  attachTool:
    '<circle cx="9" cy="6.5" r="2.3"/>' +
    '<path d="M9 8.8 V10.6"/>' +
    '<circle class="adot" cx="9" cy="12" r="1.5"/>' +
    '<path d="M6 13.5 H12 L9 20 Z"/>',

  // add a tool to the cell AND attach it (shortcut)
  addAttachTool:
    '<circle cx="8" cy="6.4" r="2.2"/>' +
    '<path d="M8 8.6 V10.2"/>' +
    '<circle class="adot" cx="8" cy="11.6" r="1.4"/>' +
    '<path d="M5 13 H11 L8 19.4 Z"/>' +
    '<path class="ac" d="M17 6.2 V10.2 M15 8.2 H19"/>',

  // attach a rigid body to a robot link
  attachBodyLink:
    '<path d="M4 7 L11 11"/>' +
    '<circle cx="4" cy="7" r="1.6"/>' +
    '<circle cx="11" cy="11" r="1.6"/>' +
    '<circle class="adot" cx="13" cy="13" r="1.4"/>' +
    '<rect x="12" y="14.5" width="7" height="6" rx="1"/>',

  // attach a rigid body (workpiece) to a tool
  attachBodyTool:
    '<path d="M6 5 H14 L10 12 Z"/>' +
    '<circle class="adot" cx="10" cy="13.3" r="1.4"/>' +
    '<rect x="6.5" y="14.8" width="7" height="6" rx="1"/>',

  // place an unattached body at a world frame
  placeBody:
    '<path d="M3 18.5 H21"/>' +
    '<rect x="9" y="10.5" width="8" height="8" rx="1"/>' +
    '<path class="ac" d="M5 18.5 V14 M5 18.5 H9.5"/>' +
    '<circle class="adot" cx="5" cy="18.5" r="1.2"/>',

  // set allowed-collision (touch) links
  touchLinks:
    '<path d="M4 6.5 L11 11.5"/>' +
    '<path d="M18 6.5 L11 11.5"/>' +
    '<circle cx="4" cy="6.5" r="1.5"/>' +
    '<circle cx="18" cy="6.5" r="1.5"/>' +
    '<circle class="adot" cx="11" cy="12" r="2"/>' +
    '<path class="ac" d="M7.5 17.5 H14.5"/>',

  /* ---------------- TARGETS ---------------- */
  // fully-constrained frame target (pose + goal ring)
  frameTarget:
    '<path d="M10 15 H18 M10 15 V7 M10 15 L13.5 18"/>' +
    '<circle class="ac" cx="10" cy="15" r="3.2"/>' +
    '<circle class="adot" cx="10" cy="15" r="1.2"/>',

  // point + axis target, rotation free
  pointAxisTarget:
    '<circle class="adot" cx="11" cy="16" r="1.8"/>' +
    '<path d="M11 16 V5 M8 8 L11 5 L14 8"/>' +
    '<path class="ac" d="M5.5 13.5 a5.5 2.6 0 0 0 11 0"/>',

  // configuration target: joints -> goal
  configTarget:
    '<circle cx="5" cy="17" r="1.5"/>' +
    '<path d="M5 17 L9 13"/>' +
    '<circle cx="9" cy="13" r="1.5"/>' +
    '<path d="M9 13 L12.5 10.5"/>' +
    '<circle class="ac" cx="16" cy="8" r="3"/>',

  // sequence of frame waypoints along a path
  frameWaypoints:
    '<path d="M3 19 L21 6" stroke-dasharray="2 2"/>' +
    '<path d="M6 16.8 H10 M6 16.8 V12.8"/>' +
    '<path d="M11.6 12.5 H15.6 M11.6 12.5 V8.5"/>' +
    '<path class="ac" d="M17.2 8.2 H21 M17.2 8.2 V4.4"/>',

  // sequence of point-axis waypoints along a path
  pointAxisWaypoints:
    '<path d="M3 19 L21 6" stroke-dasharray="2 2"/>' +
    '<circle class="dot" cx="6" cy="16.8" r="1.3"/>' +
    '<path d="M6 16.8 V12.8"/>' +
    '<circle class="dot" cx="12" cy="12.5" r="1.3"/>' +
    '<path d="M12 12.5 V8.5"/>' +
    '<circle class="adot" cx="18" cy="8.2" r="1.3"/>' +
    '<path class="ac" d="M18 8.2 V4.2"/>',

  // build a configuration from joint sliders
  robotConfig:
    '<path d="M4 7.5 H20"/><circle class="adot" cx="9" cy="7.5" r="2"/>' +
    '<path d="M4 12 H20"/><circle class="adot" cx="14" cy="12" r="2"/>' +
    '<path d="M4 16.5 H20"/><circle class="adot" cx="11" cy="16.5" r="2"/>',

  /* ---------------- PLANNING ---------------- */
  // forward kinematics: configuration -> frame
  fk:
    '<circle cx="4" cy="8" r="1.4"/>' +
    '<path d="M4 8 L7 12"/>' +
    '<circle cx="7" cy="12" r="1.4"/>' +
    '<path d="M9.5 13 H13.5 M12 11.5 L14 13 L12 14.5"/>' +
    '<path class="ac" d="M17 17 H21 M17 17 V12 M17 17 L15 19"/>',

  // inverse kinematics: frame -> configuration
  ik:
    '<path d="M4 9 H8 M4 9 V5 M4 9 L6 7"/>' +
    '<path d="M10 13 H14 M12.5 11.5 L14.5 13 L12.5 14.5"/>' +
    '<circle class="adot" cx="17" cy="9" r="1.4"/>' +
    '<path class="ac" d="M17 9 L20 13"/>' +
    '<circle class="adot" cx="20" cy="13" r="1.4"/>',

  // free-space (joint interpolated) motion -> goal
  planMotion:
    '<circle class="dot" cx="5" cy="18" r="1.6"/>' +
    '<path d="M5 18 C 6 9, 13 7, 16.5 8" stroke-dasharray="2 2"/>' +
    '<circle class="ac" cx="18" cy="8" r="2.6"/>',

  // linear cartesian motion through waypoints (hero)
  planCartesian:
    '<path d="M4 18 L9 14 L14 10 L18.5 6.5"/>' +
    '<circle class="dot" cx="4" cy="18" r="1.3"/>' +
    '<circle class="dot" cx="9" cy="14" r="1.3"/>' +
    '<circle class="dot" cx="14" cy="10" r="1.3"/>' +
    '<path class="ac" d="M15.4 5.2 L19.2 6.2 L18.2 10"/>',

  // wrap a trajectory into a named Action (card holds a path)
  trajectoryAction:
    '<rect x="3.5" y="6.5" width="17" height="11" rx="2.2"/>' +
    '<path d="M6.5 14 L9.5 11 L13 12.5 L16 9.5"/>' +
    '<circle class="dot" cx="6.5" cy="14" r="1"/>' +
    '<circle class="dot" cx="13" cy="12.5" r="1"/>' +
    '<circle class="adot" cx="16" cy="9.5" r="1.2"/>',

  // wrap a state change into a named Action (card holds a state-swap)
  stateChangeAction:
    '<rect x="3.5" y="6.5" width="17" height="11" rx="2.2"/>' +
    '<rect x="6" y="9.6" width="4" height="4.8" rx="0.7"/>' +
    '<path class="ac" d="M11 12 H14 M12.7 10.5 L14.3 12 L12.7 13.5"/>' +
    '<rect class="adot" x="15" y="9.6" width="4" height="4.8" rx="0.7"/>',

  // assemble ordered Actions into one ActionChain (threaded sequence)
  actionChain:
    '<rect x="8.5" y="5" width="11.5" height="3.5" rx="1.5"/>' +
    '<rect x="8.5" y="10.25" width="11.5" height="3.5" rx="1.5"/>' +
    '<rect x="8.5" y="15.5" width="11.5" height="3.5" rx="1.5"/>' +
    '<path class="ac" d="M5 6.75 V17.25"/>' +
    '<circle class="adot" cx="5" cy="6.75" r="1.3"/>' +
    '<circle class="adot" cx="5" cy="12" r="1.3"/>' +
    '<circle class="adot" cx="5" cy="17.25" r="1.3"/>',

  // explode a trajectory into per-point samples + data
  deconstructTrajectory:
    '<path d="M3.5 9.5 L8 7.5 L12.5 9.5 L17 6.5 L20.5 8"/>' +
    '<path d="M3 19 H21"/>' +
    '<path class="ac" d="M6 19 V15.5 M10 19 V13 M14 19 V14.5 M18 19 V12"/>',

  /* ---------------- BACKENDS ---------------- */
  // analytical (closed-form) solver chip — labelled "A"
  analyticalPlanner:
    '<rect x="5" y="6" width="14" height="12" rx="1.5"/>' +
    '<path d="M5 9.5 H3 M5 14.5 H3 M19 9.5 H21 M19 14.5 H21"/>' +
    '<path class="ac" d="M9.4 15.4 L12 8.6 L14.6 15.4"/>' +
    '<path class="ac" d="M10.45 12.9 H13.55"/>',

  // MoveIt / ROS-backed planner chip — labelled "M"
  moveitPlanner:
    '<rect x="5" y="6" width="14" height="12" rx="1.5"/>' +
    '<path d="M5 9.5 H3 M5 14.5 H3 M19 9.5 H21 M19 14.5 H21"/>' +
    '<path class="ac" d="M8.8 15.4 V8.6 L12 12.6 L15.2 8.6 V15.4"/>',

  // ROS bridge client (connection)
  rosClient:
    '<circle cx="6" cy="12" r="2.6"/>' +
    '<circle cx="18" cy="12" r="2.6"/>' +
    '<path class="ac" d="M8.6 12 H15.4"/>' +
    '<circle class="adot" cx="12" cy="12" r="1.4"/>',

  /* ---------------- ROS ---------------- */
  // publish to a topic (out)
  publish:
    '<circle cx="6.5" cy="17" r="2.4"/>' +
    '<path class="ac" d="M8.6 15.4 L12.4 11.6 M10.4 11.4 L12.6 11.4 L12.6 13.6"/>' +
    '<path d="M14.5 13 a3.4 3.4 0 0 0 -1 -5.2 M16.7 14.6 a6 6 0 0 0 -1.6 -8.8"/>',

  // subscribe to a topic (in)
  subscribe:
    '<circle cx="17.5" cy="17" r="2.4"/>' +
    '<path class="ac" d="M12 11.4 L15.6 15 M13.6 14.8 L15.8 15 L15.6 12.8"/>' +
    '<path d="M9.5 13 a3.4 3.4 0 0 1 1 -5.2 M7.3 14.6 a6 6 0 0 1 1.6 -8.8"/>',

  /* ---------------- DISPLAY ---------------- */
  // visualize the robot cell (viewport preview)
  visualize:
    '<path class="ac" d="M4 8 V4 H8 M16 4 H20 V8 M20 16 V20 H16 M8 20 H4 V16"/>' +
    '<path d="M12 8 L15 9.7 L12 11.4 L9 9.7 Z"/>' +
    '<path d="M9 9.7 V13.4 L12 15.1 L15 13.4 V9.7 M12 11.4 V15.1"/>',
};
