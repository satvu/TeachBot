// Constants
const DIR = 'https://localhost:8000/';    // Directory containing resources
const JOINTS = 7;                         // Numer of joints in Sawyer arm
const VERBOSE = true;                     // Whether or not to print everything
const BUTTON = {'back': 0, 'show': 1, 'circle': 2, 'square': 3, 'triangle': 4};
const ROBOT = 'sawyer';

/**
 * A learning module for TeachBot.
 *
 * General interpreter for module JSON files. Loads, parses, and plays instructions.
 * Example of how to use:
 * >> var m = new Module(1, main, [image,canvas_container]);
 * >> async function main() {
 * >>     m.displayOff();
 * >>     image.style.display = 'initial';
 * >>     m.start();
 * >> }
 *
 * @class
 *
 * @param {number}		module_num			The module number. Used to find JSON, audio, and text files.
 * @param {function}	main 				Function to run when all resources have been loaded.
 * @param {Array}		content_elements	List of all HTML elements that have display properties.
 */
function Module(module_num, main, content_elements) {
	if (VERBOSE) console.log(`Beginning Module ${module_num}`);

	// Instance Variables
	this.module_num = module_num;
	this.main = main;
	this.content_elements = content_elements;
	this.loaded = {'audio':false, 'text':false, 'json':false};
	
	// Initialize self to module for use in event callbacks
	self = this;

	/*******************************
	 *   Setup ROS communication   *
	 *******************************/
	this.ros = new ROSLIB.Ros({
		url: 'wss://localhost:9090'
	});
	this.ros.on('connection', function() { console.log('Connected to websocket server.');});
	this.ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
	this.ros.on('close', function() { console.log('Connection to websocket server closed.');});

	// Publishing topics
	this.audio_duration_topic = new ROSLIB.Topic({
		ros: this.ros,
		name: '/audio_duration',
		messageType: 'std_msgs/Float64'
	});
	this.wheel_subscription_topic = new ROSLIB.Topic({
		ros: this.ros,
		name: '/wheel_subscription',
		messageType: 'std_msgs/Bool'
	});
	this.GoToJointAngles = new ROSLIB.Topic({
		ros: this.ros,
		name: '/GoToJointAngles',
		messageType: ROBOT + '/GoToJointAngles'
	});
	this.joint_move = new ROSLIB.Topic({
		ros: this.ros,
		name: '/joint_move',
		messageType: ROBOT + '/joint_move'
	});
	this.interaction_control = new ROSLIB.Topic({
		ros: this.ros,
		name: '/InteractionControl',
		messageType: ROBOT + '/InteractionControl'
	});
	this.joint_angle = new ROSLIB.Topic({
		ros: this.ros,
		name: '/JointAngle',
		messageType: 'std_msgs/String'
	});
	this.joint_impedance = new ROSLIB.Topic({
		ros: this.ros,
		name: '/JointImpedance',
		messageType: ROBOT + '/JointImpedance'
	});
	this.multiple_choice = new ROSLIB.Topic({
		ros: this.ros,
		name: '/multiple_choice',
		messageType: 'std_msgs/Bool'
	})
	this.highTwo = new ROSLIB.Topic({
		ros: this.ros,
		name: '/highTwo',
		messageType: 'std_msgs/Bool'
	})
	this.adjustPoseTo = new ROSLIB.Topic({
		ros: this.ros,
		name: '/adjustPoseTo',
		messageType: ROBOT + '/adjustPoseTo'
	})
	this.closeGripper = new ROSLIB.Topic({
		ros: this.ros,
		name: '/closeGripper',
		messageType: 'std_msgs/Bool'
	})
	this.openGripper = new ROSLIB.Topic({
		ros: this.ros,
		name: '/openGripper',
		messageType: 'std_msgs/Bool'
	})
	this.GoToCartesianPose = new ROSLIB.Topic({
		ros: this.ros,
		name: '/GoToCartesianPose',
		messageType: ROBOT + '/GoToCartesianPose'
	});
	this.cuffInteraction = new ROSLIB.Topic({
		ros: this.ros,
		name: '/cuffInteraction',
		messageType: ROBOT + '/cuffInteraction'
	});
	this.check_pickup = new ROSLIB.Topic({
		ros: this.ros,
		name: '/check_pickup',
		messageType: 'std_msgs/Bool'
	})
	this.adjustPoseBy = new ROSLIB.Topic({
		ros: this.ros,
		name: '/adjustPoseBy',
		messageType: ROBOT + '/adjustPoseBy'
	})
	this.camera = new ROSLIB.Topic({
		ros: this.ros,
		name: '/camera',
		messageType: 'std_msgs/Bool'
	})

	// Subscribing topics
	this.command_complete = new ROSLIB.Topic({
		ros: this.ros,
		name: '/command_complete',
		messageType: 'std_msgs/Empty'
	})
	this.wheel_delta_topic = new ROSLIB.Topic({
		ros: this.ros,
		name: '/wheel_delta',
		messageType: 'std_msgs/Int32'
	})
	this.dev_receiver = new ROSLIB.Topic({
		ros: this.ros,
		name: '/dev_topic',
		messageType: 'std_msgs/Int32'
	})
	this.scroll_wheel_button_receiver = new ROSLIB.Topic({
		ros: this.ros,
		name: '/scroll_wheel_button_topic',
		messageType: 'std_msgs/Empty'
	})
	this.position = new ROSLIB.Topic({
		ros: this.ros,
		name: '/position',
		messageType: ROBOT + '/JointInfo'
	});
	this.pressed = new ROSLIB.Topic({
		ros: this.ros,
		name: '/scroll_wheel_pressed',
		messageType: 'std_msgs/Bool'
	});
	this.effort = new ROSLIB.Topic({
		ros: this.ros,
		name: '/effort',
		messageType: ROBOT + '/JointInfo'
	});
	this.toggle = new ROSLIB.Topic({
		ros: this.ros,
		name: '/toggle_completed',
		messageType: 'std_msgs/Empty'
	});
	this.highTwo_success = new ROSLIB.Topic({
		ros: this.ros,
		name: '/highTwo_success',
		messageType: 'std_msgs/Bool'
	});
	this.endpoint = new ROSLIB.Topic({
		ros: this.ros,
		name: '/EndpointInfo',
		messageType: ROBOT + '/EndpointInfo'
	});
	this.pickedup = new ROSLIB.Topic({
		ros: this.ros,
		name: '/pickedup',
		messageType: 'std_msgs/Bool'
	});
	this.pos_orient = new ROSLIB.Topic({
		ros: this.ros,
		name: '/pos_orient',
		messageType: 'std_msgs/String'
	});

	// Initialize dictionary
	this.dictionary = {};
	this.getEndpoint();

	/*********************
	 *   HTML Elements   *
	 *********************/
	this.ctx = canvas_obj.getContext('2d');
	canvas_obj.width = window.innerWidth*0.98;
	canvas_obj.height = window.innerHeight*0.76;
	this.ch = canvas_obj.height/100.0;
	this.cw = canvas_obj.width/100.0;
	this.robot_color = getComputedStyle(document.body).getPropertyValue('--robot-color')
	// var bar_ctx = [];
	// for(let j=0; j<JOINTS; j++) {
	//     bar_ctx.push(document.getElementById('bar' + (j+1)).getContext('2d'));
	// }

	/************************
	 *   Update Font Size   *
	 ************************/
	this.textBoxMaxHeight = document.getElementById("adjustable").scrollHeight;

	/************************
	 *   Development Mode   *
	 ************************/
	this.dev_receiver.subscribe(this.devRxCallback);

	/******************************
	 *   Setup Section Sequence   *
	 ******************************/
	var jsonPath = DIR + 'js/json/' + this.module_num + '.json';
	var jqhxr = $.getJSON(jsonPath, function(data) {
		self.sections = data.sections;
		for (let s=0; s<self.sections.length; s++) {
			self.sections[s]._textLoaded = false;
			self.sections[s]._audioLoaded = false;
		}

		self.loadTextAndAudio();
	});
	jqhxr.fail(function() {
		throw new Error('JSON file not formatted correctly. Go to ' + jsonPath + ' to learn more.')
	});
	jqhxr.done(function() {
		self.loaded['json'] = true;
		if (self.allLoaded()) { self.main(); }
	});
}

/**
 * Loads text and audio.
 */
Module.prototype.loadTextAndAudio = function() {
	// Base of directory containing text
	this.text_dir = DIR + 'text/module' + this.module_num + '/';
	
	// For each section
	for (let s=0; s<this.sections.length; s++) {

		// Load all lines in corresponding text file.
		jQuery.get(this.text_dir + this.sections[s].id + '.txt', function(data) {

			// Load text.
			self.sections[s]._textArray = data.split('\n');
			self.sections[s]._textLoaded = true;

			// Load audio.
			var audioCount = self.sections[s]._textArray.length;
			// If there are no audio files in this section, mark the section as loaded.
			if (audioCount==0) {
				self.sections[s]._textLoaded = true;
				self.sections[s]._audioLoaded = true;

			// Otherwise, load them.
			} else {
				self.sections[s]._audiofiles_mp3 = new Array(audioCount);
				self.sections[s]._audio_duration = new Array(audioCount);
				self.sections[s]._audio_duration_copy = new Array(audioCount);
				self.sections[s]._num_loaded = 0;
				for (let a=0; a<audioCount; a++) {
					self.sections[s]._audiofiles_mp3[a] = DIR + 'audio/module' + self.module_num + '/' + self.sections[s].id + '/line' + a.toString() + '.mp3';
					let audio = new Audio();
					audio.addEventListener('canplaythrough', function() {
						self.sections[s]._audio_duration[a] = audio.duration*1000;
						self.sections[s]._audio_duration_copy[a] = audio.duration*1000;
						self.sections[s]._num_loaded++;
						if (self.sections[s]._num_loaded === audioCount) {
							self.sections[s]._audioLoaded = true;

							for (let ss=0; ss<self.sections.length; ss++) {
								if (!self.sections[ss]._audioLoaded || !self.sections[ss]._textLoaded) {
									return;
								}
							}
							self.loaded['text'] = true;
							self.loaded['audio'] = true;
							if (self.allLoaded()) { self.main(); }
						}
					}, false);
					audio.src = self.sections[s]._audiofiles_mp3[a];
				}
			}

			// If all the sections' text and audio files have been loaded, mark it and check to proceed.
			for (let ss=0; ss<self.sections.length; ss++) {
				if (!self.sections[ss]._audioLoaded || !self.sections[ss]._textLoaded) {
					return;
				}
			}
			self.loaded['text'] = true;
			self.loaded['audio'] = true;
			if (self.allLoaded()) { self.main(); }
		});
	}
}

/**
 * Play audio file.
 *
 * Plays a given audio file.
 *
 * @param {string}	audioFile 	URL of audio file.
 * @param {number}	duration 	Duration of audio file in milliseconds.
 * @param {string} 	text 		Text to display on screen while playing audio.
 */
Module.prototype.play = async function(audioFile, duration, text) {
	player.src = audioFile;
	player.play();
	adjustable.innerHTML = text;
	this.pub_dur(duration/1000.0);
	if (VERBOSE) console.log('Playing ' + audioFile + ', duration ' + Math.round(duration/1000.0) + 's.');
	this.adjust_text();
}

/**
 * Publish the duration of the current audio file.
 *
 * The robot subscriber can use this information to time events.
 *
 * @param {number}	audio_duration 	The duration of the current audio file in seconds.
 */
Module.prototype.pub_dur = function(audio_duration) {
	let msg = new ROSLIB.Message({
		data: audio_duration
	});
	this.audio_duration_topic.publish(msg);
}

/**
 * Commands the steward to go to specific joint positions.
 *
 * When this function is called, the robot subscriber should command the robot to move to the position specified.
 *
 * @param {[string,Array]}	joint_angles 	Either an Array of joint angles or the name of the specific pose.
 * @param {number}			[speed_ratio=0]	Fraction of maximum speed for the robot to move.
 * @param {bool}            wait Tells the robot whether or not to wait for the audio file to be done playing
 */
Module.prototype.pub_goto = function(joint_angles, speed_ratio=0, wait=false) {
	if (typeof joint_angles === 'string') {
		var req = new ROSLIB.Message({
			name: this.hashTokeyVal(joint_angles)
		});
	} else if (joint_angles instanceof Array) {
		var req = new ROSLIB.Message({
			j0pos: joint_angles[0],
			j1pos: joint_angles[1],
			j2pos: joint_angles[2],
			j3pos: joint_angles[3],
			j4pos: joint_angles[4],
			j5pos: joint_angles[5],
			j6pos: joint_angles[6]
		});
	} else {
		throw 'goToJointAngles instruction incorrectly formatted.';
	}
	if (speed_ratio != 0) {
		req.speed_ratio = speed_ratio;
	}
	if (wait != false){
		req.wait = wait
	}
	this.GoToJointAngles.publish(req);
}

/**
 * Allows the roboto to be pushed by the user depending on the joint being specified.
 *
 * When this function is called, the robot subscriber should command the robot to unlock its joints and htese jints sshould move based on the force being felt.
 *
 * @param {[string,Array]}	joints 	Either an Array of joint(s) that determines which joints to unlock.
 * @param {string}			terminatingCondition	Condition that once reached, the program exit out of the function.
 * @param {string}          resetPOS String that states the position the robot should default to after the terminatingCondition is reached
 * @param {string}          min_thresh Specifies the minimum amount of force that requires the joint(s) to move
 * @param {string}          bias Each joint has a bias that takes into account an internal force in the robot and counteracts it
 * @param {float32}         tol Float that is used in the callback function in the teachbot script  
 */
Module.prototype.pub_move = function(joints, terminatingCondition, resetPOS, min_thresh, bias, tol = 0) {
	
	var req = new ROSLIB.Message({
		joints: this.hashTokeyVal(joints),
		terminatingCondition: this.hashTokeyVal(terminatingCondition),
		resetPOS: this.hashTokeyVal(resetPOS),
		min_thresh: this.hashTokeyVal(min_thresh),
		bias: this.hashTokeyVal(bias)
	});
	if (tol != 0) {
		req.tol = tol;
	}

	this.joint_move.publish(req);

}

Module.prototype.pub_interaction = function(position_only,orientation_x,orientation_y,orientation_z,position_x,position_y,position_z,in_end_point_frame,PASS,ways) {
	
	var req = new ROSLIB.Message({
		position_only: position_only,
		position_x: position_x,
		position_y: position_y,
		position_z: position_z,
		orientation_x: orientation_x,
		orientation_y: orientation_y,
		orientation_z: orientation_z,
		in_end_point_frame: in_end_point_frame,
		PASS: PASS,
		ways: ways
	});

	this.interaction_control.publish(req);

}

Module.prototype.pub_angle = function(angle) {
	
	var req = new ROSLIB.Message({
		data: this.hashTokeyVal(angle)
	});

	this.joint_angle.publish(req);

}

Module.prototype.pub_impedance = function(terminatingCondition,tics) {
	
	var req = new ROSLIB.Message({
		terminatingCondition: this.hashTokeyVal(terminatingCondition),
		tics: tics
	});

	this.joint_impedance.publish(req);

}

Module.prototype.pub_adjustPoseTo = function(geometry,axis,amount){
	var req = new ROSLIB.Message({
		geometry: this.hashTokeyVal(geometry),
		axis: this.hashTokeyVal(axis),
		amount: this.hashTokeyVal(amount),
	});

	this.adjustPoseTo.publish(req)
}

Module.prototype.pub_cartesian = function(position, orientation, relative_pose, joint_angles, endpoint_pose) {
	
	var req = new ROSLIB.Message({
		position: this.hashTokeyVal(position),
		orientation: this.hashTokeyVal(orientation),
		relative_pose: this.hashTokeyVal(relative_pose),
		joint_angles: this.hashTokeyVal(joint_angles),
		endpoint_pose: this.hashTokeyVal(endpoint_pose)
	});

	this.GoToCartesianPose.publish(req);

}
Module.prototype.pub_cuff = function(terminatingCondition, ways) {
	
	var req = new ROSLIB.Message({
		terminatingCondition: this.hashTokeyVal(terminatingCondition),
		ways: ways
	});

	this.cuffInteraction.publish(req);

}

Module.prototype.pub_adjustPoseBy = function(geometry,axis,amount){
	var req = new ROSLIB.Message({
		geometry: this.hashTokeyVal(geometry),
		axis: this.hashTokeyVal(axis),
		amount: this.hashTokeyVal(amount),
	});

	this.adjustPoseBy.publish(req)
}

Module.prototype.getEndpoint = function(){
	this.endpoint.subscribe(function(message) { 
    	self.dictionary['position_x'] = message.position.x;
		self.dictionary['position_y'] = message.position.y;
		self.dictionary['position_z'] = message.position.z;
		self.dictionary['orientation_x'] = message.orientation.x;
		self.dictionary['orientation_y'] = message.orientation.y;
		self.dictionary['orientation_z'] = message.orientation.z;
		self.dictionary['orientation_w'] = message.orientation.w;
		//console.log(self.dictionary)	
	});
}


/**
 * Adjusts the text size to fit in the text box.
 *
 * Adjusts the font until the text fills the most space possible in the text box or the maximum font size is reached.
 *
 * @param {string}	[max_font_size='64px']	The maximum allowable font size.
 */
Module.prototype.adjust_text = function(max_font_size='64px') {
	var resize_me = document.getElementById('adjustable');
	resize_me.style.fontSize = max_font_size;
	while (resize_me.scrollHeight > this.textBoxMaxHeight && parseInt(resize_me.style.fontSize)>2){
		resize_me.style.fontSize = parseInt(resize_me.style.fontSize) - 1 + 'px';
	}
}

/**
 * Whether all page resources are loaded.
 * 
 * Returns true iff all the values in self.loaded are true.
 */
Module.prototype.allLoaded = function() {
	for (let key in self.loaded) {
		if (!self.loaded[key]) {
			return false;
		}
	}
	return true;
}

/**
 * Hides all elements on page.
 *
 * Turns the 'display' property of all elements in self.content_elements to 'none'.
 * Optionally also clears the canvas.
 *
 * @param {boolean}	[clearCanvas=false]	Whether or not to also clear the entire canvas.
 */
Module.prototype.displayOff = function(clearCanvas=false) {
	this.content_elements.forEach(function(elem) {
		elem.style.display = 'none';
	});
	if (clearCanvas) {
		this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
	}
}

/**
 * Toggles development mode.
 *
 * Should be called whenever a message is received on the development mode topic.
 * Turns development mode on if the message is 1, off if 0.
 *
 * @param {object}	message 	ROS message containing 0 or 1.
 */
Module.prototype.devRxCallback = function(message) {
	switch (message.data) {
		case 0:
			self.devMode = false
			for (let s=0; s<self.sections.length; s++) {
				self.sections[s]._audio_duration = self.sections[s]._audio_duration_copy.slice();
			}
			if (VERBOSE) console.log('Exiting dev mode.');
			break;

		case 1:
			self.devMode = true
			for (let s=0; s<self.sections.length; s++) {
				for (let d=0; d<self.sections[s]._audio_duration.length; d++) {
				self.sections[s]._audio_duration[d] = 0;
				}
			}
			if (VERBOSE) console.log('Entering dev mode.');
			break;
	}
}

/**
 * Runs a command from the JSON file.
 *
 * Finds a command from a given address in the JSON file and performs it, then advances to the next command.
 * The command address should be formatted as an array with the following elements:
 *   0: {string}  Name of section.
 *   1: {number}  Index of instruction within section.
 *   The following elements are only valid if the instruction is of type 'if' or 'while', and may proceed indefinitely:
 *     If the parent instruction is of type 'if':
 *       n  : {boolean} Whether to follow the 'if_true' or 'if_false' instructions next.
 *       n+1: {number}  Index of the child instruction.
 *     Else if the parent instruction is of type 'while':
 *       n  : {number}  Index of the child instruction.
 *
 * @param {Array}	[instructionAddr=['intro',0]]	Address of instruction to play.
 */
Module.prototype.start = async function(instructionAddr=['intro',0]) {
	this.thisSection;
	for (let s=0; s<this.sections.length; s++) {
		if (this.sections[s].id === instructionAddr[0]) {
			this.thisSection = this.sections[s];
			break;
		}
	}
	if (this.thisSection === undefined) {
		throw `There is no section with id ${instructionAddr[0]}`;
	}

	var instructions = this.thisSection.instructions;
	this.instructionSets = [JSONcopy(instructions)];

	if (instructionAddr.length == 1) {
		console.log('Reached End of Module.');
		return;
	}

	var instr = instructions[instructionAddr[1]];
	for (let instructionAddrLevel=2; instructionAddrLevel<instructionAddr.length; instructionAddrLevel++) {
		if (instr.type === 'if') {
			instructionAddrLevel++;
			if (instructionAddr[instructionAddrLevel-1]) {
				instructions = instr.if_true;
			} else {
				instructions = instr.if_false;
			}
		} else if (instr.type === 'while') {
			instructions = instr.instructions;
		} else {
			throw `Compound instruction addressing not available for instructions of type ${instr.type}`;
		}
		this.instructionSets.push(JSONcopy(instructions));
		instr = instructions[instructionAddr[instructionAddrLevel]];
	}

	if (instr.hasOwnProperty('skippable') && instr.skippable && self.devMode) {
		this.start(this.getNextAddress(instructionAddr));
	} else {
		switch(instr.type) {
			case 'arc':
				if (instr.hasOwnProperty('shoulder_arc')){
					canvas_container.style.display = 'initial';
					arc3pt(this.ctx,33*this.cw,95*this.ch,27*this.cw,48*this.ch,28*this.cw,2*this.ch,false);
				}

				if (instr.hasOwnProperty('wrist_arc')){
					this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
                	arc3pt(this.ctx,28*this.cw,13*this.ch,38*this.cw,57*this.ch,64*this.cw,46*this.ch,true);
				}

				if (instr.hasOwnProperty('shoulderNew_arc')){
					arc3pt(this.ctx,49*this.cw,94*this.ch,43*this.cw,64*this.ch,42*this.cw,14*this.ch,false);
				}

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'adjustPoseBy':
				checkInstruction(instr, ['geometry', 'axis', 'amount'], instructionAddr);

				this.pub_adjustPoseBy(instr.geometry, instr.axis, instr.amount)

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'adjustPoseTo':
				checkInstruction(instr, ['geometry', 'axis', 'amount'], instructionAddr);

				this.pub_adjustPoseTo(instr.geometry, instr.axis, instr.amount)

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'balls':
				var ballA = {x:25*this.cw, y:83*this.ch, fillStyle: 'BlueViolet'};
				var ballB = {x:22*this.cw, y:47*this.ch, fillStyle: this.robot_color};
				var ballC = {x:21*this.cw, y:18*this.ch, fillStyle: 'DarkGreen'};
				var ballR = 10*this.ch;

				if (instr.hasOwnProperty('ballAB')) {
					this.displayOff();
					canvas_container.style.display = 'initial';
					this.ctx.font = Math.round(5*this.cw) +  'px Raleway';
					draw_ball(this.ctx,ballA.x,ballA.y,ballR,ballA.fillStyle,'A');  // A
					draw_ball(this.ctx,ballB.x,ballB.y,ballR,ballB.fillStyle,'B');  // B
				} 

				if (instr.hasOwnProperty('ballC')) {
					draw_ball(this.ctx,ballC.x,ballC.y,ballR,ballC.fillStyle,'C');
				} 

				if (instr.hasOwnProperty('ballR')) {
					this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
					draw_ball(this.ctx, 53*this.cw, 89*this.ch, ballR, this.robot_color);
				} 

				if (instr.hasOwnProperty('ballMotor')) {
					this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
					draw_ball(this.ctx, 21*this.cw, 18*this.ch, ballR, this.robot_color);
				}

				if (instr.hasOwnProperty('ballR_new')) {
					this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
					draw_ball(this.ctx, 27*this.cw, 88*this.ch, ballR, this.robot_color);
				} 

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'bar_both':
				this.displayOff();

				var barL = {axisLeft: 3*this.cw, axisRight: 12*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw, fillStyle: this.robot_color};
				var barR = {axisLeft: 15*this.cw, axisRight: 24*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw, fillStyle: 'BlueViolet'};

				canvas_container.style.display = 'initial';
                draw_bar(this.ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                draw_bar(this.ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);

                this.position.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.j0, message.j5);
                    self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
                    draw_bar(self.ctx, (-message.j0*180/Math.PI+176)*0.9, 174, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                    draw_bar(self.ctx, message.j5*180/Math.PI+170, 340, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'bar_A':
				var ballA = {x:25*this.cw, y:83*this.ch, fillStyle: 'BlueViolet'};
				var barA = {axisLeft: 32*this.cw, axisRight: 41*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw};

				draw_bar(this.ctx,0.6,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A');

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'bar_B':
				var ballB = {x:22*this.cw, y:47*this.ch, fillStyle: this.robot_color};
				var barB = {axisLeft: 44*this.cw, axisRight: 53*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw};

				draw_bar(this.ctx,1,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B');

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'bar_C':
				var ballC = {x:21*this.cw, y:18*this.ch, fillStyle: 'DarkGreen'};
				var barC = {axisLeft: 56*this.cw, axisRight: 65*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw};

				draw_bar(this.ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C');

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'bar_shoulder':
				this.displayOff();

				var barL = {axisLeft: 3*this.cw, axisRight: 12*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw, fillStyle: this.robot_color};

				canvas_container.style.display = 'initial';
                draw_bar(this.ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);

                this.position.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.j0);
                    self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
                    draw_bar(self.ctx, (-message.j0*180/Math.PI+176)*0.9, 174, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                   });

                this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
					
				});

				break;

			case 'bar_wrist':
				clearInterval(encoder);
				this.displayOff();

				var barR = {axisLeft: 15*this.cw, axisRight: 24*this.cw, maxHeight: 87*this.ch, antiWidth: 0.8*this.cw, fillStyle: 'BlueViolet'};

				canvas_container.style.display = 'initial';
                draw_bar(this.ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);

                this.position.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.j5);
                    self.ctx.clearRect(0,0,100*self.cw,100*self.ch);
                    draw_bar(self.ctx, message.j5*180/Math.PI+170, 340, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
					
				});

				break;

			case 'camera':
				var req = new ROSLIB.Message({
					data: true
				});

				console.log('Getting camera info')

				this.camera.publish(req);

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break

			case 'check_pickup':
				var req = new ROSLIB.Message({
					data: true
				});

                this.check_pickup.publish(req);

                this.pickedup.subscribe(async function(message) {
                	if (VERBOSE) console.log('Picked up: ' + message.data);
					if (message.data == true) {
						wheel_val = 1
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.pickedup.unsubscribe();
						self.pickedup.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					} else{
						wheel_val = 2
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.pickedup.unsubscribe();
						self.pickedup.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'clearRect':
				this.ctx.clearRect(0,0,100*m.cw,100*m.ch);

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'closeGripper':
				var req = new ROSLIB.Message({
					data: true
				});

                this.closeGripper.publish(req);

                this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'cuff_interaction':
				checkInstruction(instr, ['terminatingCondition', 'ways'], instructionAddr);

				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				this.pub_cuff(instr.terminatingCondition, instr.ways)

				this.pos_orient.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url,message.data)
				});

				this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.ctx.clearRect(3, 300, 403, 1100)
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'drawBin':
				checkInstruction(instr, ['number'], instructionAddr);
				var binW = 17*cw;
				var binH = 13*ch;
				switch (instr.number) {
					case 1:
						// this.ctx.strokeRect(56*this.cw, 40*this.ch, binW, binH);
						this.ctx.strokeRect(62*this.cw, 40*this.ch, binW, binH);
						break;
					case 2:
						this.ctx.strokeRect(83*this.cw, 33*this.cw, binW, binH);
						break;
				}

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'drawBox':
				checkInstruction(instr, ['number'], instructionAddr);
				console.log('Drawing box')
				var boxW = 14*this.cw;
				var boxH = 10*this.ch;
				var corner_size = 0.01561*this.cw;
				this.ctx.fillStyle = "#7c2629";
				var x;
				var y;
				switch (instr.number) {
					case 1:
						// x = 83*this.cw;
						// y = 77*this.ch;
						// x = 77*this.cw;
						// y = 73*this.ch;
						x = 77*this.cw;
						y = 90*this.ch;
						this.ctx.strokeRect(x, y, boxW, boxH);
						break;
					case 2:
						// x = 70*this.cw;
						// y = 74*this.ch;
						x = 62*this.cw;
						y = 75*this.ch;
						this.ctx.strokeRect(x, y, boxH, boxW);
						break;
					case 3:
						// x = 47*this.cw;
						// y = 82*this.ch;
						x = 47*this.cw;
						y = 75*this.ch;
						this.ctx.strokeRect(x, y, boxH, boxW);
						break;
					case 4:
						// x = 41*this.cw;
						// y = 40*this.ch;
						// x = 39*this.cw;
						// y = 40*this.ch;
						x = 62*this.cw;
						y = 16*this.ch;
						this.ctx.strokeRect(x, y, boxW, boxH);
						break;
				}

				this.ctx.fillRect(x, y , corner_size, corner_size);
				this.ctx.fillRect(x+ boxW - corner_size, y, corner_size, corner_size);
				this.ctx.fillRect(x, y + boxH - corner_size, corner_size, corner_size);
				this.ctx.fillRect(x +boxW - corner_size, y + boxH - corner_size, corner_size, corner_size);

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'drawPosOrien':
				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url)

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'encode':
				self.displayOff();
				canvas_container.style.display = 'initial';
                this.ctx.clearRect(0,0,100*this.cw,100*this.ch);
                var encoder_moving_part_url = DIR + 'images/moving_part.png';
				var encoder_still_part_url = DIR + 'images/still_part.png';
				var encoder_motor_url = DIR + 'images/motor_body.png';
				var encoder_motor_circle_url = DIR + 'images/motor_circle.png';
                encoder = draw_encoder(this.ctx, encoder_moving_part_url, encoder_still_part_url, encoder_motor_url, encoder_motor_circle_url);

                this.start(self.getNextAddress(instructionAddr));

                break;

           
			case 'goToJointAngles':
				checkInstruction(instr, ['joint_angles'], instructionAddr);

				if (instr.hasOwnProperty('speed_ratio')) {
					this.pub_goto(instr.joint_angles, instr.speed_ratio);
				} else if (instr.hasOwnProperty('wait')){
					this.pub_goto(instr.joint_angles, 0, instr.wait);
				} else {
					this.pub_goto(instr.joint_angles);
				}
				
				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'goToCartesianPose':
				checkInstruction(instr, ['position','orientation','relative_pose','joint_angles','endpoint_pose'], instructionAddr);

				this.pub_cartesian(instr.position, instr.orientation, instr.relative_pose, instr.joint_angles, instr.endpoint_pose)

				this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'highTwo':
				var req = new ROSLIB.Message({
					data: true
				});

				this.highTwo.publish(req)

				this.highTwo_success.subscribe(async function(message) {
                	if (VERBOSE) console.log('High two: ' + message.data);
					if (message.data == true) {
						wheel_val = 1
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.highTwo_success.unsubscribe();
						self.highTwo_success.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					} else{
						wheel_val = 2
						if (instr.hasOwnProperty('store_answer_in')) {
							self.dictionary[instr.store_answer_in] = wheel_val;
						}
						self.highTwo_success.unsubscribe();
						self.highTwo_success.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					}
				});


				break;

			case 'hokeypokey':
				var hokeypokey_out_url = DIR + 'images/hokeypokey_out.png';
				var hokeypokey_in_url = DIR + 'images/hokeypokey_in.png';
				var hokeypokey_rot1_url = DIR + 'images/hokeypokey_rot1.png';
				var hokeypokey_rot2_url = DIR + 'images/hokeypokey_rot2.png';

				if (instr.hasOwnProperty('out')){
					image.src = hokeypokey_out_url;
				}
				if (instr.hasOwnProperty('in')){
					image.src = hokeypokey_in_url;
				}
				if (instr.hasOwnProperty('rot1')){
					image.src = hokeypokey_rot1_url;
				}
				if (instr.hasOwnProperty('rot2')){
					image.src = hokeypokey_rot2_url;
				}

				this.start(self.getNextAddress(instructionAddr));

				break;

			case 'if':
				checkInstruction(instr, ['conditional','if_true'], instructionAddr);

				if (eval(this.hashTokeyVal(instr.conditional))) {
					instructionAddr.push(true);
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else if (instr.hasOwnProperty('if_false')) {
					instructionAddr.push(false);
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else {
					this.start(this.getNextAddress(instructionAddr));
				}

				break;

			case 'image':
				var welcome_image_url = DIR + 'images/Welcome.png';
				this.displayOff();

                image.src = welcome_image_url;
                image.style.display = 'initial';

                this.start(this.getNextAddress(instructionAddr));

				break;

			case 'interaction_control':
				checkInstruction(instr, ["position_only","orientation_x","orientation_y","orientation_z","position_x","position_y","position_z", "in_end_point_frame", "PASS", "ways"], instructionAddr);

				var orient_bw_url = DIR + 'images/orientation_bw.png';
				var orient_color_url = DIR + 'images/orientation_color.png';
				var position_bw_url = DIR + 'images/position_bw.png';
				var position_color_url = DIR + 'images/position_color.png';

				// var imgData = this.ctx.getImageData(0, 0, self.cw, self.ch);

				//var imgData = canvas_obj.getContext('2d');

                this.pub_interaction(instr.position_only, instr.orientation_x, instr.orientation_y, instr.orientation_z, instr.position_x, instr.position_y, instr.position_z, instr.in_end_point_frame, instr.PASS, instr.ways);

                this.pos_orient.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					draw_pos_orien(self.ctx,3,300,400,position_color_url,position_bw_url, orient_color_url, orient_bw_url,message.data)
				});

                if (instr.hasOwnProperty('wait')){
                	this.pressed.subscribe(async function(message) {
	                	if (VERBOSE) console.log('Pressed: ' + message.data);
						if (message.data == true) {
							self.ctx.clearRect(3, 300, 403, 1100)
							// imgData.drawImage(canvas_obj,0,0);
							// self.ctx.putImageData(imgData, 0, 0)
							self.pressed.unsubscribe();
							self.pressed.removeAllListeners();
							self.start(self.getNextAddress(instructionAddr));
						}	
					});
                } else{
                	this.start(self.getNextAddress(instructionAddr));
                }

                break;

            case 'interaction_projection':
            	protractor_table.style.display = 'initial';

            	var bar_ctx = [];
				for(let j=0; j<JOINTS; j++) {
				    bar_ctx.push(document.getElementById('bar' + (j+1)).getContext('2d'));
				}

                this.position.subscribe(async function(message) {
                    if (VERBOSE) console.log('Received: ' + message.j0, message.j1, message.j2, message.j3, message.j4, message.j5, message.j6);
                    var locations = [message.j0, message.j1, message.j2, message.j3, message.j4, message.j5, message.j6]
                    for (let p=1; p<=locations.length; p++) {
                        document.getElementById('protractor' + p).value = '' + (100*locations[p-1]);
                        let cw = bar_ctx[p-1].canvas.width/100.0;
                        let ch = bar_ctx[p-1].canvas.height/100.0;
                        bar_ctx[p-1].clearRect(0,0,100*cw,100*ch);
                        draw_bar(bar_ctx[p-1], locations[p-1], 3.15,9*cw,91*cw,50*ch,6*cw, self.robot_color);
                        document.getElementById('bar' + p).value = '' + (100*locations[p-1]);
                    }
                });

                this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
					
				});

                break;

            case 'joint_angle':
            	checkInstruction(instr, ["angle"], instructionAddr);

            	this.pub_angle(instr.angle)

            	this.command_complete.subscribe(function(message) {
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

            	break;

            case 'joint_impedance':
            	checkInstruction(instr, ["terminatingCondition","tics"], instructionAddr);

            	this.pub_impedance(instr.terminatingCondition,instr.tics)

      			this.start(self.getNextAddress(instructionAddr));

            	break;

			case 'joint_move':
				checkInstruction(instr, ["joints","terminatingCondition","resetPOS","min_thresh","bias","listen"], instructionAddr);

				if (instr.hasOwnProperty('tol')) {
					this.pub_move(instr.joints, instr.terminatingCondition, instr.resetPOS, instr.min_thresh, instr.bias, instr.tol);
				} else {
					this.pub_move(instr.joints, instr.terminatingCondition, instr.resetPOS, instr.min_thresh, instr.bias);
				}

				if (instr.listen == false){
					if(instr.hasOwnProperty('wait')){
						this.pressed.subscribe(async function(message) {
		                	if (VERBOSE) console.log('Pressed: ' + message.data);
							if (message.data == true) {
								self.pressed.unsubscribe();
								self.pressed.removeAllListeners();
								self.start(self.getNextAddress(instructionAddr));
							}	
						});
					} else{
						this.start(self.getNextAddress(instructionAddr));
					}
				} else{
					if(instr.hasOwnProperty('wait')){
						this.toggle.subscribe(async function(message) {
							self.toggle.unsubscribe();
							self.toggle.removeAllListeners();
							self.start(self.getNextAddress(instructionAddr));
						});
					} else{
						this.command_complete.subscribe(function(message) {
							self.command_complete.unsubscribe();
							self.command_complete.removeAllListeners();
							self.start(self.getNextAddress(instructionAddr));
						});
					}
				}

                break;

			case 'log':
				checkInstruction(instr, ['message'], instructionAddr);

				console.log(instr.message);

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'multiple_choice':
				this.displayOff();
                canvas_container.style.display = 'initial';
                var multi_choice_url = DIR + 'images/sized_cuff.png';

                display_choices(m.ctx, ['Motors','Buttons','Cameras','Encoders','Wheels'], multi_choice_url);

                var req = new ROSLIB.Message({
					data: true
				});

                this.multiple_choice.publish(req);

                this.wheel_delta_topic.subscribe(async function(message) {
					if (VERBOSE) console.log('Button pressed:' + message.data);
					wheel_val = message.data
					if (instr.hasOwnProperty('store_answer_in')) {
						self.dictionary[instr.store_answer_in] = wheel_val;
					}
					self.wheel_delta_topic.unsubscribe();
					self.wheel_delta_topic.removeAllListeners();
					self.displayOff(true);
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'multiple_choice2':
                var req = new ROSLIB.Message({
					data: true
				});

                this.multiple_choice.publish(req);

                this.wheel_delta_topic.subscribe(async function(message) {
					if (VERBOSE) console.log('Button pressed:' + message.data);
					wheel_val = message.data
					if (instr.hasOwnProperty('store_answer_in')) {
						self.dictionary[instr.store_answer_in] = wheel_val;
					}
					self.wheel_delta_topic.unsubscribe();
					self.wheel_delta_topic.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'openGripper':
				var req = new ROSLIB.Message({
					data: true
				});

                this.openGripper.publish(req);

                this.command_complete.subscribe(function(message) {
                	console.log('Command completed');
					self.command_complete.unsubscribe();
					self.command_complete.removeAllListeners();
					self.start(self.getNextAddress(instructionAddr));
				});

				break;

			case 'play':
				checkInstruction(instr, ['audio_index','delay'], instructionAddr);
				if (instr.hasOwnProperty('//')) {
					if (!instr['//'].startsWith('#') && instr['//'] !== this.thisSection._textArray[instr.audio_index]) {
						console.warn(`Play instruction at ${instructionAddr} has a mismatch between the "//" tag and the actual text displayed.`);
					}
				} else {
					if (VERBOSE) console.warn('It is recommended that all "play" instructions have an accompanying "//" tag with the played text for readability.');
				}

				var audio_index = instr.audio_index;
				if (typeof audio_index === 'string') {
					audio_index = eval(this.hashTokeyVal(audio_index));
				}

				this.play(this.thisSection._audiofiles_mp3[audio_index], this.thisSection._audio_duration[audio_index], this.thisSection._textArray[audio_index]);
				if (instr.delay) {
					await sleep(this.thisSection._audio_duration[audio_index]);
				}

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'playVideo':
				this.displayOff();
				animator.style.display = 'initial';
				animator.play();

				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'pressed_button':
				this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.start(self.getNextAddress(instructionAddr));
					}
				});

				break;

			case 'proceed':
				checkInstruction(instr, ['to_section'], instructionAddr);

				this.start([instr.to_section, 0]);

				break;

			case 'projection':
				this.displayOff();
                canvas_container.style.display = 'initial';

                this.position.subscribe(async function(message) {
                    if (VERBOSE) console.log(message.j1);
                    draw_goal(self.ctx, 100, message.j1*400+100)
                });

                this.pressed.subscribe(async function(message) {
                	if (VERBOSE) console.log('Pressed: ' + message.data);
					if (message.data == true) {
						self.position.unsubscribe();
						self.position.removeAllListeners();
						self.pressed.unsubscribe();
						self.pressed.removeAllListeners();
						self.displayOff();
						self.start(self.getNextAddress(instructionAddr));
					}
					
				});


				break;

			case "refresh":
				m.displayOff(true);

                image.style.display = 'initial';

                this.start(this.getNextAddress(instructionAddr));

                break;

			case 'reset':
				this.displayOff();

				this.ctx.clearRect(0,0,100*this.cw,100*this.ch);

                canvas_container.style.display = 'initial';

                this.start(this.getNextAddress(instructionAddr));

				break;

			case 'scrollWheelInput':
				this.displayOff();
				canvas_container.style.display = 'initial';
				run_odometer = true;
				canvas_obj.width = canvas_obj.width;
				init_odometer(this.ctx);

				this.pub_goto('joint_buttons')

				var req = new ROSLIB.Message({
					data: true
				});
				
				this.wheel_subscription_topic.publish(req);

				this.wheel_delta_topic.subscribe(async function(message) {
					if (VERBOSE) console.log('Rx: ' + message.data);
					wheel_val+= message.data;
				});

				this.scroll_wheel_button_receiver.subscribe(async function(message) {
					if (VERBOSE) console.log('Scroll wheel button pressed.');
					if (instr.hasOwnProperty('store_answer_in')) {
						self.dictionary[instr.store_answer_in] = wheel_val;
					}
					self.wheel_delta_topic.unsubscribe();
					self.wheel_delta_topic.removeAllListeners();
					self.scroll_wheel_button_receiver.unsubscribe();
					self.scroll_wheel_button_receiver.removeAllListeners();
					cancel_odometer();
					self.displayOff(true);
					self.start(self.getNextAddress(instructionAddr));
				});
				break;

			case 'set':
				checkInstruction(instr, ['key','val'], instructionAddr);

				if (VERBOSE) console.log(`Setting #${instr.key} to ${instr.val}`);
				if (typeof instr.val === 'string') {
					this.dictionary[instr.key] = eval(this.hashTokeyVal(instr.val));
				} else {
					this.dictionary[instr.key] = instr.val;
				}
				//console.log(self.dictionary)
				this.start(this.getNextAddress(instructionAddr));
				break;

			case 'show_camera':

				var cv_image = new Image();
				cv_image.onload = function() {
					console.log('Displaying camera')
		            self.ctx.drawImage(cv_image, 20, 70)

		            self.start(self.getNextAddress(instructionAddr));
				};
				cv_image.src = DIR + 'images/cv_image.png';

				break;

			case 'update':

				this.getEndpoint()

				this.start(this.getNextAddress(instructionAddr));

				break; 

			case 'while':
				checkInstruction(instr, ['conditional'], instructionAddr);
		
				if (eval(this.hashTokeyVal(instr.conditional))) {
					console.log('conditions')
					instructionAddr.push(0);
					this.start(instructionAddr);
				} else {
					console.log('moving on')
					this.start(this.getNextAddress(instructionAddr));
				}
			
				break;
				
			default:
				throw `Did not recognize instruction of type ${instr.type}`;
		}
	}
}

/**
 * Replaces all references with a '#' with the value they reference from the dictionary.
 *
 * For each instance of '#' in a given string, this function replaces the '#' and the following variable reference with the value it references
 * from the dictionary.
 *
 * Example:
 *  >> module.dictionary.a = 314;
 *  >> console.log(module.hashTokeyVal('I have #a apples.'));
 *     I have 314 apples.
 *
 * @param {string}	str	A string to process.
 *
 * @return {object}	The processed string with references replaced.
 */
Module.prototype.hashTokeyVal = function(str) {
	return str.replace(/#[a-z_0-9]+/gi, key => this.dictionary[key.substring(1)].toString());
}

/**
 * Increments the current instruction address.
 *
 * If the current instructions list is not yet completed, simply increments the last element.
 * Otherwise, backs up one parent instruction.
 * If the module is completed, returns an Array with one element: the ID of the last section.
 *
 * @param {Array}	instructionAddr 	See play().
 *
 * @returns {Array}	Address of next instruction to play.
 */
Module.prototype.getNextAddress = function(instructionAddr) {
	if (instructionAddr[instructionAddr.length-1] < this.instructionSets[this.instructionSets.length-1].length-1) {
		instructionAddr[instructionAddr.length-1]++;
		return instructionAddr;
	} else {
		this.instructionSets.pop();
		instructionAddr.pop();
		if (typeof instructionAddr[instructionAddr.length-1] === 'boolean') {
			instructionAddr.pop();
			return this.getNextAddress(instructionAddr);
		} else {
			return instructionAddr;
		}
	}
}


/**
 * Checks an instruction to make sure it has the required properties.
 *
 * If the given instruction does not have the required properties, throws and error to alert the user.
 *
 * @param {object}	instruction 	The instruction object from the JSON file to be checked.
 * @param {Array}	properties		A list of strings of the required properties.
 * @param {Array}	addr 			Address of given instruction so it can alert the user if there is a problem.
 */
function checkInstruction(instruction, properties, addr) {
	for (let p=0; p<properties.length; p++) {
		if (!instruction.hasOwnProperty(properties[p])) {
			throw `Instruction ${instruction.type} at ${addr} requires property ${properties[p]}.`
		}
	}
}