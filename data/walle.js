
// Shortcut function to access HTML elements
function get(elt) {
  return document.getElementById(elt);
}

// Handle to GUI controls elements
var urlTxt = get('urlTxt');
var msgTxt = get('msgTxt');
var logTxt = get('logTxt');

var connectBtn = get('connectBtn');
var sendBtn = get('sendBtn');

var controllerStateTxt = get('controllerStateTxt');
var robotStateTxt = get('robotStateTxt');

var compassSvg = get('compass');


// A generic joystick class for interactive X/Y control with feedback
class Joystick {
  constructor(cfg) {
    this.cfg = cfg;
    this.canvas = get(this.cfg.canvasId);
    this.canvas.width = this.cfg.width;
    this.canvas.height = this.cfg.height;
    this.ctx = this.canvas.getContext('2d');
    this.position = {x:0, y: 0};    
    this.feedback = {x:0, y:0};
    this.held = false;

    this.canvas.addEventListener('touchstart', this.moveStart.bind(this));
    this.canvas.addEventListener('touchmove', this.move.bind(this));
    this.canvas.addEventListener('touchend', this.moveEnd.bind(this));
    this.canvas.addEventListener('touchcancel', this.moveEnd.bind(this));

    this.canvas.addEventListener('mousedown', this.moveStart.bind(this));
    this.canvas.addEventListener('mousemove', this.move.bind(this));
    this.canvas.addEventListener('mouseup', this.moveEnd.bind(this));
    this.canvas.addEventListener('mouseout', this.moveEnd.bind(this));

    this.draw();
  }

  // Convert screen position to -range .. +range coordinates
  getPos(evt) {
    if (evt.changedTouches != undefined) { // convert touch into click-like event
      evt = evt.changedTouches[0];
    }
    var w = this.canvas.width;
    var h = this.canvas.height;
    var r = this.cfg.radius;

    var rect = evt.target.getBoundingClientRect();
    var evtx = evt.clientX - rect.left;
    var evty = evt.clientY - rect.top;
    var x = map(constrain(evtx, r, w - r), r, w - r, this.cfg.range.x[0], this.cfg.range.x[1]);
    var y = map(constrain(evty, r, h - r), h - r, r, this.cfg.range.y[0], this.cfg.range.y[1]);

    return {x: Math.round(x), y: Math.round(y)};
  }

  // Redraw canvas and update controller state
  update() {
    this.draw();
    inputChange();
  }

  // Start dragging the joystick
  moveStart(evt) {
    evt.preventDefault();
    this.held = true;
    this.position = this.getPos(evt);
    this.update();
  }

  // ongoing move
  move(evt) {
    if (this.held) {
      evt.preventDefault();
      this.position = this.getPos(evt);
      this.update();
    }
  }

  // end of move
  moveEnd(evt) {
    evt.preventDefault();
    if (this.held) {
      this.held = false;
      if (this.cfg.returnToCenter) {
        this.position = {x:0, y: 0};
        this.update();
      }
    }
  }

  // Redraw the canvas
  draw() {
    var w = this.canvas.width;
    var h = this.canvas.height;
    var r = this.cfg.radius;
    var c = this.ctx;

    // Draw background
    c.fillStyle = '#fed';
    c.fillRect(0, 0, w, h);
    // Draw rounded rectangle
    c.lineJoin = 'round';
    c.lineWidth = 2*r;
    c.strokeStyle = c.fillStyle = this.cfg.backgroundColor || '#e0e0e0';
    c.strokeRect(r, r, w-2*r, h-2*r);
    c.fillRect(r, r, w-2*r, h-2*r);

    // Write text
    c.strokeStyle = c.fillStyle = this.cfg.feedbackColor  || '#404040';
    c.font = '15px Arial';
    c.textAlign = 'center';
    c.fillText(this.cfg.label, w/2, r);
    c.font = '10px Arial';
    c.textAlign = 'left';
    c.fillText('x: ' + this.position.x.toFixed(0) + this.cfg.unit.x + ' (' + this.feedback.x.toFixed(0) + this.cfg.unit.x + ')', 0, r+15);
    c.fillText('y: ' + this.position.y.toFixed(0) + this.cfg.unit.y + ' (' + this.feedback.y.toFixed(0) + this.cfg.unit.y + ')', 0, r+30);

    var indicatorX = map(this.feedback.x, this.cfg.range.x[0], this.cfg.range.x[1], r, w - r);
    var indicatorY = map(this.feedback.y, this.cfg.range.y[0], this.cfg.range.y[1], h - r, r);
    var joystickX = map(this.position.x, this.cfg.range.x[0], this.cfg.range.x[1], r, w - r);
    var joystickY = map(this.position.y, this.cfg.range.y[0], this.cfg.range.y[1], h - r, r);
    
    // Special case for 1-dof joystick
    if (this.cfg.range.x[0] == this.cfg.range.x[1]) {
      joystickX = indicatorX = w/2;
    }
    if (this.cfg.range.y[0] == this.cfg.range.y[1]) {
      joystickY = indicatorY = h/2;
    }
    
    // Draw feedback indicator (shows current robot state)
    c.beginPath();
    c.fillStyle = this.cfg.feedbackColor || '#ffc653';
    c.arc(indicatorX, indicatorY, 1*r, 0, 2 * Math.PI);
    c.fill();

    // Draw joystick (shows controller state)
    c.beginPath();
    c.fillStyle = this.cfg.joystickColor || '#ffaa00';
    c.arc(joystickX, joystickY, .8*r, 0, 2 * Math.PI);
    c.fill();
  }
}

// Create joystick instances 
var navJoystick = new Joystick({
    canvasId: 'canvasNavJoystick', width: 120, height: 120, radius: 12, label: 'Navigation', 
    range: {x: [-100, 100], y: [100,-100]}, unit: {x:'%', y:'%'}, returnToCenter: true, 
    backgroundColor: '#e0e0e0', feedbackColor: '#805000', joystickColor: '#ffaa00',
  });

var headJoystick = new Joystick({
    canvasId: 'canvasHeadJoystick', width: 200, height: 100, radius: 12, label: 'Head', 
    range: {x:[90, -90], y:[90, -90]}, unit: {x:'°', y:'°'},
    backgroundColor: '#e0e0e0', feedbackColor: '#802800', joystickColor: '#ff4900',
  });

var leftArmJoystick = new Joystick({
    canvasId: 'canvasLeftArmJoystick', width: 120, height: 120, radius: 12, label: 'Left arm', 
    range: {x:[90, -90], y:[90, -90]}, unit: {x:'°', y:'°'},
    backgroundColor: '#e0e0e0', feedbackColor: '#184018', joystickColor: '#308030', 
  });

var rightArmJoystick = new Joystick({
    canvasId: 'canvasRightArmJoystick', width: 120, height: 120, radius: 12, label: 'Right arm', 
    range: {x:[90, -90], y:[-90, 90]}, unit: {x:'°', y:'°'},
    backgroundColor: '#e0e0e0', feedbackColor: '#184018', joystickColor: '#308030',    
  });

var leftHandJoystick = new Joystick({
    canvasId: 'canvasLeftHandJoystick', width: 30, height: 120, radius: 12, label: 'L hand', 
    range: {x:[0, 0], y:[90, -90]}, unit:{x:'', y:'°'},
    backgroundColor: '#e0e0e0', feedbackColor: '#181840', joystickColor: '#303080',
  });

var rightHandJoystick = new Joystick({
    canvasId: 'canvasRightHandJoystick', width: 30, height: 120, radius: 12, label: 'R Hand', 
    range: {x:[0, 0], y:[-90, 90]}, unit:{x:'', y:'°'},
    backgroundColor: '#e0e0e0', feedbackColor: '#181840', joystickColor: '#303080',
  });

var joysticks = [navJoystick, headJoystick, leftArmJoystick, leftHandJoystick, rightArmJoystick, rightHandJoystick];

const AXIS_X = 0, AXIS_Y = 1;
var dofTable = [
  {id: 0, name: 'Fwd',  joystick: navJoystick,      axis: AXIS_Y},
  {id: 1, name: 'Turn', joystick: navJoystick,      axis: AXIS_X},
  {id: 2, name: 'HdUp', joystick: headJoystick,     axis: AXIS_Y},
  {id: 3, name: 'HdLR', joystick: headJoystick,     axis: AXIS_X},
  {id: 4, name: 'LAUp', joystick: leftArmJoystick,  axis: AXIS_Y},
  {id: 5, name: 'LALR', joystick: leftArmJoystick,  axis: AXIS_X},
  {id: 6, name: 'LH',   joystick: leftHandJoystick, axis: AXIS_Y},
  {id: 7, name: 'RAUp', joystick: rightArmJoystick, axis: AXIS_Y},
  {id: 8, name: 'RALR', joystick: rightArmJoystick, axis: AXIS_X},
  {id: 9, name: 'RH',   joystick: rightHandJoystick,axis: AXIS_Y},
];

var attitudes = ['Yaw', 'Pitch', 'Roll'];

/*
var setposTextList = degreesOfFreedom.map(dofName => {return get('cmd' + dofName)});
var feedbackTextList = degreesOfFreedom.map(dofName => {return get('feedback' + dofName)});
*/
var feedbackAttitudeList = attitudes.map(attName => {return get('feedback' + attName)});

var socket = undefined;

// Current state of the remote controller
var controllerState = {move:[0,0],servos:[0,0,0,0]};

// Last known state of the robot we're controlling
var robotState = {move:[0,0],servos:[0,0,0,0]};

// Utility: map (linear interpolation)
function map(x, x0, x1, y0, y1) {
  return y0 + (y1 - y0) * (1.0 * x - x0) / (x1 - x0);
}

// Utility: constrain a variable inside bounds
function constrain(x, x0, x1) {
  return Math.min(Math.max(x, x0), x1);
}

// Show logs in debug console, as well as in text box
function log(msg) {
  var now = new Date();
  var timestamp = ('00' + now.getHours()).slice(-2) + ':'
      + ('00' + now.getMinutes()).slice(-2) + ':'
      + ('00' + now.getSeconds()).slice(-2) + '.' 
      + ('000' + now.getMilliseconds()).slice(-3);
  msg = '[' + timestamp + ']' + msg;
  // print on console
  console.log(msg);
  // Append to log textarea, scroll to last line
  logTxt.value += '\n' + msg;
  logTxt.scrollTop = logTxt.scrollHeight;
}

// This is called whenever a joystick is moved
function inputChange() {
  dofTable.forEach(dof => {
    var value = [dof.joystick.position.x, dof.joystick.position.y][dof.axis];
    if (dof.id < 2) { // controllerState.move
      controllerState.move[dof.id] = value;
    } else { // controllerState.servos
      controllerState.servos[dof.id - 2] = value;
    }
  });

  controllerStateSpan.innerHTML = JSON.stringify(controllerState);
}

// Parse messages from robot
function handleMessage(msg) {
  log('          <- ' + msg);
  if (msg[0] == '{') {
    // msg contains JSON formatted data
    robotState = JSON.parse(msg);
    robotStateSpan.innerHTML = JSON.stringify(robotState);

    if (robotState.attitude != undefined) {
      for (var i = 0; i < 3; i++) {
        feedbackAttitudeList[i].innerHTML = robotState.attitude[i] + '&deg;';
      }
      compassSvg.setAttribute('transform', 'rotate(' + robotState.attitude[0] + ')');
    }

    dofTable.forEach(dof => {
      var value = 0.;
      if (dof.id < 2) {
        value = robotState.move[dof.id];
      } else {
        value = robotState.servos[dof.id - 2];
      }
      if (dof.axis == AXIS_X) {
        dof.joystick.feedback.x = value;
      } else {
        dof.joystick.feedback.y = value;
      }      
    });
    joysticks.forEach(joystick => {joystick.draw();});
  }
  // reply with current controller state
  socket.send(JSON.stringify(controllerState));
}

function send(msg) {
  if (socket == undefined) {
    log('! Not connected');
  } else {
    log('-> ' + msg);
    socket.send(msg);
  }
}

connectBtn.onclick = function(evt) {
  var url = urlTxt.value;
  if (socket != undefined) {
    log('! Closing websocket');
    socket.close();
  } else {
    log('! Attempting to connect to ' + url);
    socket = new WebSocket(url);

    socket.onmessage = function(evt) {
      handleMessage(evt.data);
    }

    socket.onclose = function(evt) {
      log('! Socket closed');
      socket = undefined;
      connectBtn.innerHTML = 'Connect';
      connectBtn.classList.remove('red');
      connectBtn.classList.add('green');
    }

    socket.onerror = function(evt) {
      log('! Socket error' + JSON.stringify(evt));
      socket.close();
    }

    connectBtn.innerHTML = 'Disconnect';
    connectBtn.classList.add('red');
    connectBtn.classList.remove('green');
  }
}

