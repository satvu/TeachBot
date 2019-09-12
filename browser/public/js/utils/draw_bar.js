/**
 * Draw a bar chart.
 *
 * Draws a bar chart object on a canvas. 
 *
 * @param {object}  ctx_in          The context of the canvas on which to draw.
 * @param {number}  val             The value to display on the bar chart.
 * @param {number}  maxVal          The maximum value displayable on the bar chart.
 * @param {number}  axisLeft        The x position of the left corner of the x-axis of the bar chart in pixels.
 * @param {number}  axisRight       The x position of the right corner of the x-axis of the bar chart in pixels.
 * @param {number}  maxHeight       The maximum height of the bar chart in pixels.
 * @param {number}  antiWidth       The distance of the edges of the bar chart from each side of the x-axis.
 * @param {string}  fillStyle       The color of the ball's fill.
 * @param {string}  [label=null]    The label to display under the bar chart.
 */
function draw_bar(ctx_in, val, maxVal, axisLeft, axisRight, maxHeight, antiWidth, fillStyle, label=null) {
    ctx_in.strokeStyle = '#000';
    ctx_in.lineWidth = 0;
    ctx_in.beginPath();
    ctx_in.moveTo(axisLeft+antiWidth,maxHeight);
    ctx_in.lineTo(axisLeft+antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx_in.lineTo(axisRight-antiWidth,maxHeight-maxHeight*val/maxVal);
    ctx_in.lineTo(axisRight-antiWidth,maxHeight);
    ctx_in.lineTo(axisLeft+antiWidth,maxHeight);
    ctx_in.fillStyle = fillStyle;
    ctx_in.fill();

    ctx_in.strokeStyle = '#FFF';
    ctx_in.lineWidth = 5;
    ctx_in.beginPath();
    ctx_in.moveTo(axisLeft,maxHeight);
    ctx_in.lineTo(axisRight,maxHeight);
    ctx_in.stroke();
    ctx_in.strokeStyle = '#000';
    ctx_in.lineWidth = 0;
    ctx_in.closePath();
    if (label!==null) {
        fontSize = parseInt(/([0-9]+)px/g.exec(ctx_in.font)[1]);
        ctx_in.fillText(label,axisLeft+(axisRight-axisLeft)/2-fontSize*label.length/2,maxHeight+1.2*fontSize);
    }
/*    ctx_in.closePath();*/
}