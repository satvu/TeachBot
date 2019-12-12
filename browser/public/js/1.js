'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [image, animator]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'intro';
async function main() {
    m.displayOff();
    image.style.display = 'none';
    m.start([start_seq,0]);
}