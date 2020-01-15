/**
 * Changes the current graphics mode.
 *
 * Sets the graphic mode to one of the following:
 * - 'image': Displays an image.
 * - 'video': Plays a video.
 * - 'canvas': Displays the canvas and draws objects on it from the drawings array.
 * - 'multiple choice': Displays the multiple choice graphic with custom options.
 * - 
 *
 * @param {object}  instr            A parameterized instruction imported from the JSON file.
 * @param {object}  instructionAddr  The address of the current instruction.
 */
Module.prototype.set_graphic_mode = function(instr, instructionAddr) {
	checkInstruction(instr, ['mode'], instructionAddr);
	this.displayOff();
	this.graphic_mode = instr.mode;

	switch (instr.mode) {
		case 'image':
			checkInstruction(instr, ['location'], instructionAddr);

			image.src = DIR + instr.location;
			image.style.display = 'initial';

			break;

		case 'video':
			animator.style.display = 'initial';
			animator.play();
			
			break;

		case 'canvas':
			canvas_container.style.display = 'initial';
			if (instr.hasOwnProperty('clear') && instr.clear) {
				this.drawings = [];
			}

			this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
				self.drawCanvas(timestamp);
			});
			
			break;

		default:
			throw `Graphic mode ${instr.mode} is not supported.`;
	}
	
	this.start(this.getNextAddress(instructionAddr));
};

Module.prototype.drawCanvas = function(timestamp) {
	this.ctx.clearRect(0,0,100*this.cw,100*this.ch);

	this.drawings.forEach(function(obj, ind) {
		switch (obj.shape) {
			case 'ball':
				var cx = eval(self.hashTokeyVal(obj.cx));
				var cy = eval(self.hashTokeyVal(obj.cy));
				var r = eval(self.hashTokeyVal(obj.r));
				var fillStyle = obj.fillStyle;

				draw_ball(self.ctx, cx, cy, r, fillStyle, obj.label);

				break;

			case 'bar':
				var val = eval(self.hashTokeyVal(obj.val));
				var maxVal = eval(self.hashTokeyVal(obj.maxVal));
				var axisLeft = eval(self.hashTokeyVal(obj.axisLeft));
				var axisRight = eval(self.hashTokeyVal(obj.axisRight));
				var maxHeight = eval(self.hashTokeyVal(obj.maxHeight));
				var antiWidth = eval(self.hashTokeyVal(obj.antiWidth));
				var fillStyle = eval(self.hashTokeyVal(obj.fillStyle));

				draw_bar(self.ctx, val, maxVal, axisLeft, axisRight, maxHeight, antiWidth, fillStyle, obj.label);

				break;

			case 'arc':
				var x1 = eval(self.hashTokeyVal(obj.x1));
				var y1 = eval(self.hashTokeyVal(obj.y1));
				var x2 = eval(self.hashTokeyVal(obj.x2));
				var y2 = eval(self.hashTokeyVal(obj.y2));
				var x3 = eval(self.hashTokeyVal(obj.x3));
				var y3 = eval(self.hashTokeyVal(obj.y3));
				var ccw = eval(self.hashTokeyVal(obj.ccw));

				arc3pt(self.ctx, x1, y1, x2, y2, x3, y3, ccw);

				break;
		}
	});

	this.canvas_frame_req = window.requestAnimationFrame(function(timestamp) {
		self.drawCanvas(timestamp);
	});
}