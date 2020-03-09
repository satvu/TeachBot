'use strict'

const module_num = 42;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image,canvas_container]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
async function main() {
    m.displayOff();
    image.style.display = 'initial';
    m.start([start_seq,0]);
}