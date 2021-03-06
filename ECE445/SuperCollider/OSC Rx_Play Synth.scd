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
s.boot;								// Boot server
s.meter;							// Open level meter
s.scope;							// Open Scope
s.freqscope;						// Open Frequency Scope
s.plotTree;							// Open Node Tree
)

NetAddr.langPort					// Show the current port SC is listening on in post window

// Define Synth Object
(
SynthDef.new(\fmSynth,{
	arg carHz=600.0, modHz=200.0, modAmp=200.0, amp=0.2, pan=0.0, gate=0;
//	atk=0.01, rel=1;
	var sig, mod, env;
	env = EnvGen.kr(Env.adsr, gate);//, doneAction:2);	// doneAction:2 frees the synth object when amp is 0
	mod = SinOsc.ar(modHz, mul:modAmp);
	sig = SinOsc.ar(carHz + mod) * amp * env;
	sig = Pan2.ar(sig, pan);
	Out.ar(0, sig);
}).add;
)

//mapping functions
//mapTo180 = { arg x; x/180};
//mapTo90 = { arg x; x/90};
//mapToFreq = {arg data; 220 * (((int)data * (1/15)) + 12)};
//mapToAmp = {arg data; ((data) / 600)};

// Recieving OSC messages and play synth
(
~synth1 = Synth.new(\fmSynth);
OSCdef.new(\recv, {
	arg msg;
	var sig, freq, amp, pan, gate, x;
	// Expected message format: \path, arg1, arg2, arg3
<<<<<<< HEAD
	msg.postln;                // Post OSC message to window
	freq = msg[1]/180 * 440;    // Map freq range from [-180, 180] to [220, 880]
	amp = msg[2]/180;                    // Map amp range from  [-180, 180] to [0, 0.8]
	pan = msg[3]/180;                                // Map pan from [-180, 180] to [-1, 1]
	x = [freq, amp, pan];				    // Assign freq, amp, and pan to array x
	x.postln;					    // Post [freq, amp, pan] values to window
	~synth1.set(\carHz, freq, \amp, amp, \pan, pan);
=======
	msg[3].postln;		// Post OSC message to window
	//Lag.kr(msg,0.1);		// Smoothing function
	gate = msg[4];			// Gate trigger

	// Synth frequency
	if (
		(msg[1] <= 90.0) && (msg[1] >= -90.0),							// Conditional
		{freq = 220.0 * ((((msg[1])) * (24/180)) + 12).midiratio},		// Function if true
		{																// Function if false
			if (msg[1] > 90.0,											// Conditional
				{freq = 220 * (36 - (12 * msg[1] / 90.0)).midiratio},	// Function if true
				{freq = 220 * (-12 - (12 * msg[1] / 90.0)).midiratio}	// Function if false
			)
		}
	);
	amp = ((msg[2] * (0.8/180)) + 0.4);			// Map amp range from [-180, 180] to [0, 0.6]
	if (
		(msg[3] <= 90.0) && (msg[3] >= -90.0),			// Conditional
		{pan = ((msg[3] / -90.0))},						// Function if true
		{												// Function if false
			if (msg[3] > 90.0,							// Conditional
				{pan = (-2 + (msg[3] / 90.0));},		// Function if true
				{pan = (2 + (msg[3] / 90.0));}		// Function if false
			)
		}
	);
	// pan = LinExp.kr(pan, -1, 1, -1, 1);
	//pan.linexp(-1,1,-1,1);
	x = pan;				// Assign freq, amp, and pan to array x
	x.postln;						// Post [freq, amp, pan] values to window
	~synth1.set(\carHz, freq, \amp, amp, \pan, pan, \gate, gate);
>>>>>>> 3478d58626741af3303936b364bc24c56cce0e3c
}, '/testAddr');
)

(
~synth1.free;				// Stop Synth from playing

OSCdef.freeAll;				// Release all OSC objects
)

