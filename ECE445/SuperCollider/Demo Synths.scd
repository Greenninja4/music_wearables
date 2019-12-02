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
~sawSynth = Synth.new(\sawtone);

// Execute function
~sawSynth.set(\freq, 800, \amp, 0.5);

// Free Synth object
~sawSynth.free;

//////////////////////////////////////// AM Synth ////////////////////////////////////////
(
SynthDef.new(\am,{
	arg carHz=600, modHz=200, modAmp=1, amp=0.2, pan=0;
	var car, mod, env;
	mod = SinOsc.ar(MouseX.kr(0.01*modHz,modHz,1), mul:modAmp);
	//car = SinOsc.ar(MouseY.kr(0.5*carHz,2*carHz,1) + mod) * amp;
	car = VarSaw.ar((carHz * MouseY.kr(0,12,0).midiratio), mul:amp * mod);
	car = Pan2.ar(car, pan);
	Out.ar(0, car);
}).add;
)

~amSynth = Synth.new(\am)

~amSynth.free


//////////////////////////////////////// FM Synth ////////////////////////////////////////
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

//////////////////////////////////////// Sequencer ////////////////////////////////////////
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

//////////////////////////////////////// Random synth ////////////////////////////////////////
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

//////////////////////////////////////// Sawtooth ////////////////////////////////////////
(
SynthDef(\sawsynth,
	{
		arg freq=400, atk=0.01, rel=1, amp=0.2, hrm=4, out=0;
		var sig, env;
		env = Env.perc(atk, rel).kr(2);
		sig = Saw.ar([freq, freq * 0.2.midiratio]);
		sig = RLPF.ar(sig, (freq * hrm).clip(20, 20000), 0.5);
		sig = sig * env * amp;
		Out.ar(out, sig);
	}
).add;
)

(
Pbindef(\sawpatrn,
	\instrument, \sawsynth,
	\dur, 0.2,
	\scale, Scale.minorPentatonic,
	\degree, Pseq([0,-1,-2,-3,-4,-5], inf),
	\amp, 0.2,
	\atk, 0.01,
	\rel, 0.3,
	\out, 0,
).play
)

Pbindef(\sawpatrn).set(\hrm, 50);
Pbindef(\sawpatrn).stop;


//////////////////////////////////////// IMU Synth 1 ////////////////////////////////////////
(
SynthDef.new(\IMUSynth_1,{
	arg freq=600.0, detune=0.2, amp=0.2, pan=0.0, gate=1,
	fmHz=200.0, fmAmp=0.0, lpf=15000,
	atk=0.01, rel=1.0, curv=(-4.0);
	var sig, fm, am, env, detuneCtrl;
	env = EnvGen.kr(
		Env.adsr(atk,releaseTime:rel, curve:curv),
		gate,
		doneAction:2);	// doneAction:2 frees the synth object when env finishes
	fm = SinOsc.ar(fmHz, mul:fmAmp);
	detuneCtrl = LFNoise1.kr(0.1!8).bipolar(detune).midiratio;
	sig = Saw.ar((freq + fm) * detuneCtrl);
	sig = sig * env;
	sig = Splay.ar(sig, (1.0 - pan.abs), center:pan);
	sig = LPF.ar(sig, lpf.clip(20, 20000), amp);
	Out.ar(0, sig);
}).add;
)

(
Pbindef(\IMU_pat,
	\instrument, \IMUSynth_1,
	\dur, 0.2,
	\scale, Scale.minorPentatonic,
	\degree, Pseq([0,-1,-2,-3,-4,-5], inf),
	\amp, 0.2,
	\pan, Pwhite(-1.0,1.0, inf),
	\lpf, Pwhite(50,15000, inf),
	\out, 0,
).play
)
Pbindef(\IMU_pat).set(\rel,1.0);
Pbindef(\IMU_pat).stop;

~imuSynth1 = Synth.new(\IMUSynth_1);
~imuSynth1.set(\gate, 1);
~imuSynth1.set(\freq, 300, \pan, 0.0);
~imuSynth1.set(\lpf, 15000);
~imuSynth1.set(\gate, 0);


s.options.numOutputBusChannels_(2);
s.boot;
s.meter;
s.quit;

Env.adsr(0.02, 0.2, 0.25, 1, 1, -5).test(2).plot;
Env.adsr(0.001, 0.2, 0.25, 1, 1, -4).test(2).plot;
Env.adsr(0.001, 0.2, 0.25, 1, 1, -4).test(10).plot;    // release after 0.45 sec