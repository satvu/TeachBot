/**
 * Draws and animates the on-screen odometer.
 *
 * Displays the odometer on the screen and animates it when wheel_val is changed.
 *
 * @param {object}  timestamp     See window.requestAnimationFrame().
 * @param {object}  ctx_in        The context in which to animate.
 * @param {boolean} run_odometer  Whether or not the odometer should be animated.
 * @param {number}  dial_spacing  How far apart to space the dial notches.
 * @param {number}  dial_pos      The position of the dial.
 * @param {number}  dial_speed    The animation speed of the dial.
 * @param {object}  back_plate    Image of the dial back plate.
 * @param {object}  center_dial   Image of the dial center dial.
 * @param {object}  indicator     Image of the dial indicator.
 * @param {object}  shadow        Image of the dial shadow.
 * @param {object}  mask          Image of the dial mask.
 */
function draw_odometer(timestamp, ctx_in, run_odometer, dial_spacing, dial_pos, dial_speed, back_plate, center_dial, indicator, shadow, mask) {
    if (run_odometer===true){

        canvas = ctx_in.canvas;
        //the goal numbers
        var desired_val = -wheel_val*dial_spacing + Math.PI; //The theta coordinate position of the dial inverted this
        //console.log("animation wheel_val", wheel_val) //commented out to reduce console garbage
        var offset = (dial_pos%(2*Math.PI)); //inverted both of these
        var small_offset = (dial_pos%(dial_spacing));

        var img_scale = canvas.height * .7
        var imgposh =canvas.height/2- img_scale/2;
        var imgposw = canvas.width/2- img_scale/2;

        if (dial_pos<0){
          var generated_min = Math.floor(-dial_pos/dial_spacing);
        } else {
          var generated_min = Math.ceil(-dial_pos/dial_spacing);
        }
        //var generated_max =  Math.floor((2*Math.PI-offset)/dial_spacing);
        //the number of numbers that need to be
        if (dial_pos<desired_val-.01){
          dial_pos = dial_pos+2*Math.PI/dial_speed;
        }else if (dial_pos>desired_val+.01){
          dial_pos = dial_pos - 2*Math.PI/dial_speed;
        }else{
          //when dial_pos = desired_val;
          generated_min = wheel_val-5;
          small_offset = 0;
        }
        ctx_in.globalCompositeOperation = 'destination-over';
        ctx_in.clearRect(0, 0, canvas.width, canvas.height); // clear canvas
        ctx_in.fillStyle = 'rgba(0, 0, 0, 0.4)';
        ctx_in.strokeStyle = 'rgba(0, 153, 255, 0.4)';
        ctx_in.save()
        ctx_in.drawImage(indicator,imgposw,imgposh, width = img_scale, height = img_scale);
        ctx_in.translate(canvas.width/2,canvas.height/2);
        ctx_in.rotate(offset)
        ctx_in.drawImage(center_dial,-img_scale/2,-img_scale/2, width = img_scale, height = img_scale);
        ctx_in.restore();
        ctx_in.drawImage(mask,imgposw,imgposh, width = img_scale, height = img_scale);
        ctx_in.save();
        ctx_in.translate(canvas.width/2,canvas.height/2);
        ctx_in.rotate(small_offset);
        ctx_in.font =  38*img_scale/400+ "px Courier New";
        ctx_in.textAlign="center";
        //drawing the numbers in
        var i;
        for (i=0;i<=(8);i++){
          ctx_in.save()
          ctx_in.rotate(0.03); //adjust this value if the rotation is just slightly off
          ctx_in.translate(0,98*img_scale/400)
          ctx_in.rotate(Math.PI);
          ctx_in.fillText(generated_min+i,0, 0);
          ctx_in.restore()
          ctx_in.rotate(dial_spacing)
        }

        ctx_in.restore();
        ctx_in.drawImage(shadow,imgposw, imgposh, width = img_scale, height = img_scale);
        ctx_in.drawImage(back_plate,imgposw, imgposh, width = img_scale, height = img_scale);
        odometer_req = window.requestAnimationFrame(function(timestamp) {
          draw_odometer(timestamp, ctx_in, run_odometer, dial_spacing, dial_pos, dial_speed, back_plate, center_dial, indicator, shadow, mask);
        });


  } else {
    ctx_in.clearRect(0, 0, ctx_in.canvas.width, ctx_in.canvas.width);
  }
}

var wheel_val = 5; //how far apart numbers are
var dial_spacing = Math.PI/5; //the rotation we want
var desired_val = wheel_val*dial_spacing-Math.PI; //this way, desired points to the bottom of the wheel
var dial_pos = 0;
var dial_speed = 200;
var num_notches= 5

var run_odometer = true;

var odometer_req;

function init_odometer(ctx_in) {
  canvas_obj.width = canvas_obj.width
  odometer_req = window.requestAnimationFrame(function(timestamp) {
    draw_odometer(timestamp, ctx_in, run_odometer, dial_spacing, dial_pos, dial_speed, back_plate, center_dial, indicator, shadow, mask);
  });
}

function cancel_odometer() {
  window.cancelAnimationFrame(odometer_req);
  canvas_obj.width = canvas_obj.width;
}

// TODO: Find a better way to load images in Module.js
var back_plate = new Image();
var center_dial = new Image();
var indicator = new Image();
var shadow = new Image();
var mask = new Image();
back_plate.src = 'https://localhost:8000/images/back_plate.png';
indicator.src = 'https://localhost:8000/images/Indicator.png';
shadow.src = 'https://localhost:8000/images/shadow.png';
mask.src = 'https://localhost:8000/images/mask.png';
center_dial.src = 'https://localhost:8000/images/center_dial.png';