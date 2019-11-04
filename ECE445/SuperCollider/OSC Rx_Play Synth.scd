/* Helpful keyboard shortcuts:
   cmd D = lookup documentation for text under cursor
   shft cmd D = enter text to look up in documentation
   cmd . = Stop audio
   shft cmd P = clear post window
   opt cmd T = show node tree
   cmd M = show meter
   shft cmd M = show scope
   opt cmd M = show freq scope
   Run each code cluster enclosed by () by pressing cmd enter
   Run each line of code by pressing shift enter
*/

/*
   Use this code to test incoming OSC messages from IMU
*/

// Quick setup of session
(
s.boot;                             // Boot server
s.meter;                            // Open level meter
s.scope;                            // Open Scope
s.freqscope;                        // Open Frequency Scope
s.plotTree;                         // Open Node Tree
)

NetAddr.langPort                    // Show the current port SC is listening on in post window

// Define Synth Object
(
SynthDef.new(\fmSynth,{
	arg carHz=600, modHz=200, modAmp=200, amp=0.2, pan=0,
	atk=0.01, rel=1;
	var sig, mod, env;
	//env = EnvGen.kr(Env.perc(atk, rel), doneAction:2);
	mod = SinOsc.ar(modHz, mul:modAmp);
	sig = SinOsc.ar(carHz + mod) * amp;// * env;
	sig = Pan2.ar(sig, pan);
	Out.ar(0, sig);
}).add;
)

// Recieving OSC messages and play synth
(
~synth1 = Synth.new(\fmSynth);
OSCdef.new(\recv, {
	arg msg;
	var sig, freq, amp, pan, x;
	// Expected message format: \path, arg1, arg2, arg3
	//msg.postln;                // Post OSC message to window
	freq = 220 * ((msg[1] * (1/15)) + 12).midiratio;    // Map freq range from [-180, 180] to [220, 880]
	amp = ((msg[2] + 180) / 600);                       // Map amp range from  [-180, 180] to [0, 0.8]
	pan = msg[3] / 180;                                 // Map pan from [-180, 180] to [-1, 1]
	x = [freq, amp, pan];				    // Assign freq, amp, and pan to array x
	x.postln;					    // Post [freq, amp, pan] values to window
	~synth1.set(\carHz, freq, \amp, amp, \pan, pan);
}, '/testAddr');
)

~synth1.free;                       // Stop Synth from playing

OSCdef.freeAll;                     // Release all OSC objects
