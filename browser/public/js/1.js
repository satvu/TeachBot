'use strict'

const module_num = 1;         //The module number

/*******************************
 *       Construct Module      *
 *******************************/
var m = new Module(module_num, main, [welcome, animator]);

/**************************
 *   Main Functionality   *
 **************************/
var start_seq = 'feedback';
async function main() {
    m.displayOff();
    welcome.style.display = 'none';
    m.start([start_seq,0]);
}