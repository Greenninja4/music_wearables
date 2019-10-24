/* Helpful keyboard shortcuts:
   cmd D = lookup documentation for text under cursor
   shft cmd D = enter text to look up in documentation
   cmd . = Stop audio
   shft cmd P = clear post window
   opt cmd T = show node tree
   cmd M = show meter
   shft cmd M = show scope
   opt cmd M = show freq scope
*/

(
s.boot;                             // Boot server
s.meter;                            // Open level meter
s.scope;                            // Open Scope
s.freqscope;                        // Open Frequency Scope
s.plotTree;                         // Open Node Tree
)

// Define a synth object
(
SynthDef.new(\sawtone, {
    // Synth object arguments
	arg freq=400, amp=0.1;

	// Initialize local variables
	var sig, freqCtrl;

	// Generate an array of frequencies that deviate at a rate of 2 Hz
	// around the value of freq by semitones rather than Hz
	// And use x-axis of mouse to controll frequency of synth
	freqCtrl = MouseX.kr(0.5 * freq,2 * freq, 1) * LFNoise1.kr(2!8).range(-0.25, 0.25).midiratio;
	sig = VarSaw.ar(freqCtrl,mul:MouseY.kr(0,amp,0).poll.clip(0,1));    // Sawtooth generator

	// Spread array across stereo field
	sig = Splay.ar(sig);
	Out.ar(0, sig);                // Send sig to outputs 0 and 1
}).add;
)

// Assign synth node to environment variable
~sawSynth = Synth.new(\sawtone)

// Execute function
~sawSynth.set(\freq, 800, \amp, 0.5)

// Free Synth object
~sawSynth.free;


// FM Synth
(
SynthDef.new(\fm,{
	arg carHz=600, modHz=200, modAmp=200, amp=0.2, pan=0;
	var car, mod, env;
	mod = SinOsc.ar(MouseX.kr(0.1*modHz,10*modHz,1), mul:modAmp);
	//car = SinOsc.ar(MouseY.kr(0.5*carHz,2*carHz,1) + mod) * amp;
	car = SinOsc.ar((carHz * MouseY.kr(0,12,0).midiratio).poll + mod, mul:amp);
	car = Pan2.ar(car, pan);
	Out.ar(0, car);
}).add;
)

~fmSynth = Synth.new(\fm)

~fmSynth.free

(
~fmSynth = Pbind(
	\instrument, \fm,
	\dur, 1/8,
	\amp, Pexprand(0.1, 0.5),
	\atk, Pexprand(0.001, 0.05),
	\rel, Pexprand(0.05, 1.2),
	\pan, Pwhite(-1.0, 1.0)
).play;
)

// Sequencer
(
SynthDef.new(\fmperc,{
	arg carHz=600, modHz=200, modAmp=200, amp=0.2, pan=0,
	atk=0.01, rel=1;
	var car, mod, env;
	env = EnvGen.kr(Env.perc(atk, rel), doneAction:2);
	mod = SinOsc.ar(modHz, mul:modAmp);
	car = SinOsc.ar(carHz + mod) * amp * env;
	car = Pan2.ar(car, pan);
	Out.ar(0, car);
}).add;
)

(
~fmPercSynth = Pbind(
	\instrument, \fmperc,
	\dur, 1/8,
	\carHz, Pexprand(20, 10000),
	\modHz, Pexprand(20, 10000),
	\modAmp, Pwhite(0, 10000),
	\amp, Pexprand(0.05, 0.1),
	\atk, Pexprand(0.001, 0.05),
	\rel, Pexprand(0.05, 1.2),
	\pan, Pwhite(-1.0, 1.0)
).play;
)

~fmPercSynth.stop


// TO DO: Figure out how to control Pbind dur externally

// Random synth
(
SynthDef.new(\RndmSyn, {
	arg amp=0.3, speed=8, detune=0.25;
	var sig, freqCtrl, freq, rng_min, rng_max;
	rng_min = 110 * MouseY.kr(0, 36).midiratio;
	freq = LFNoise0.kr(MouseY.kr(speed - 3,speed + 3,0)).range(rng_min,rng_min * (MouseX.kr(0,24)).midiratio);
	freqCtrl = freq * LFNoise1.kr(speed!8).range(-1 * detune,detune).midiratio;
	sig = VarSaw.ar(freqCtrl, 0, mul:amp);
	sig = Splay.ar(sig,MouseX.kr(1,0),center:MouseY.kr(-1,1));
	Out.ar(0,sig);
}).add;
)

~rndmSyn = Synth.new(\RndmSyn);
~rndmSyn.free;