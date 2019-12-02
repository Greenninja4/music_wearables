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
SynthDef.new(\IMUSynth_1,{
	arg freq=600.0, detune=0.2, amp=0.2, pan=0.0, gate=0,
	fmHz=200.0, fmAmp=0.0, lpf=15000,
	atk=0.01, rel=1.0, curv=(-4.0);
	var sig, fm, am, env, detuneCtrl;
	env = EnvGen.kr(
		Env.adsr(atk,releaseTime:rel, curve:curv),
		gate,);
//		doneAction:2);	// doneAction:2 frees the synth object when env finishes
	fm = SinOsc.ar(fmHz, mul:fmAmp);
	detuneCtrl = LFNoise1.kr(0.1!8).bipolar(detune).midiratio;
	sig = SinOsc.ar((freq + fm) * detuneCtrl);
//	sig = Saw.ar((freq + fm) * detuneCtrl);
	sig = sig * env;
	sig = Splay.ar(sig, (1.0 - pan.abs), center:pan);
	sig = LPF.ar(sig, lpf.clip(80, 15000), amp);
	Out.ar(0, sig);
}).add;
)

// Recieving OSC messages and play synth
(
~synth1 = Synth.new(\IMUSynth_1);

/////////////////////////////////////// Device 1 ///////////////////////////////////////
OSCdef.new(\dev01, {
	arg msg;
	var sig, freq, cutoff, mod, gate, x;
	// Expected message format: \path, arg1, arg2, arg3
	// msg[2].postln;		// Post OSC message to window
	gate = msg[4];			// Gate trigger

	// Synth frequency
	freq = 440 * msg[1].fold(-90.0, 90.0).linlin(-90.0, 90.0, 12, -12).midiratio;

	// LPF cutoff frequency
	cutoff = msg[2].linlin(-90.0,90.0,15000,30).clip(20, 20000);

	//Synth Modulator
	mod = msg[3].linlin(-180.0, 180.0, -219.98, 439.99).fold(0.01, 220.0);

	// Post values
	x = [msg[0], freq, cutoff, mod, gate];				// Assign freq, cutoff, and mod to array x
	x.postln;						// Post [freq, cutoff, mod] values to window

	// Send values to synth object
	~synth1.set(\freq, freq, \lpf, cutoff, \fmHz, mod, \gate, gate);
}, '/device1');


/////////////////////////////////////// Device 2 ///////////////////////////////////////
OSCdef.new(\dev02, {
	arg msg;
	var sig, amp, mod2, pan, x;
	// Expected message format: \path, arg1, arg2, arg3
	// msg[3].postln;		// Post OSC message to window

	// Synth amplitude
	amp = msg[1].fold(-90.0, 90.0).linlin(-90.0,90.0,0,0.8);

	// Synth modulator 2
	mod2 = msg[2].linlin(-90.0,90.0,200.0,0.0);

	//Synth panning
	pan = msg[3].linlin(-180.0, 180.0, 2.0, -2.0).fold(-1.0,1.0);

	// Post values
	//	x = [msg[0], amp, mod2, pan];				// Assign amp, mod2, and pan to array x
	// x.postln;						// Post [amp, mod2, pan] values to window

	// Send values to synth object
	~synth1.set(\amp, amp, \fmAmp, mod2, \pan, pan);
}, '/device2');
)

// Free synth and OSC objects
(
~synth1.free;				// Stop Synth from playing

OSCdef.freeAll;				// Release all OSC objects
)

