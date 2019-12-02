(
//1. server config
s = Server.local;
s.options.outDevice_(
	"Built-in Output"
);
/*s.options.numOutputBusChannels_(2);
s.options.inDevice_("Built-in Microph");
s.options.numInputBusChannels_(2);
s.options.sampleRate_(44100);
s.options.memSize_(2.pow(20));
s.newBusAllocators;
ServerBoot.removeAll;
ServerTree.removeAll;
ServerQuit.removeAll;*/

//2. initialize global variables
t = TempoClock.new(90/60).permanent_(true);
~out = 0;
//~path = PathName(thisProcess.nowExecutingPath).parentPath++"buffers/";


//3. define piece-specific functions
~makeBuffers = {
/*	b = Dictionary.new;
	PathName(~path).entries.do{
		arg subfolder;
		b.add(
			subfolder.folderName.asSymbol ->
			Array.fill(
				subfolder.entries.size,
				{
					arg i;
					Buffer.read(s, subfolder.entries[i].fullPath);
				}
			)
		);
	};*/

	//10 wavetables with increasing complexity
	~wt_sig = 10.collect({
		arg i;

		//random number of envelope segments
		var numSegs = i.linexp(0,9,4,40).round;

		Env(
			//env always begins and ends with zero
			//inner points are random from -1.0 to 1.0
			[0]++({1.0.rand}.dup(numSegs-1) * [1,-1]).scramble++[0],

			//greater segment duration variety in higher-index wavetables
			{exprand(1,i.linexp(0,9,1,50))}.dup(numSegs),

			//low-index wavetables tend to be sinusoidal
			//high index wavetables tend to have sharp angles and corners
			{[\sine,0,exprand(1,20) * [1,-1].choose].wchoose([9-i,3,i].normalizeSum)}.dup(numSegs)
		).asSignal(1024);
	});

	//load into 10 buffers in wavetable format
	~wt_buf = Buffer.allocConsecutive(10, s, 2048, 1, {
		arg buf, index;
		buf.setnMsg(0, ~wt_sig[index].asWavetable);
	});
};

~makeBusses = {
	~bus = Dictionary.new;
	~bus.add(\reverb -> Bus.audio(s,2));
};

~cleanup = {
	s.newBusAllocators;
	ServerBoot.removeAll;
	ServerTree.removeAll;
	ServerQuit.removeAll;
};

~makeNodes = {
	s.bind({
		~mainGrp = Group.new;
		~reverbGrp = Group.after(~mainGrp);
		~reverbSynth = Synth.new(
			\reverb,
			[
				\predelay, 0.1,
				\revtime, 1.8,
				\lpf, 4500,
				\in, ~bus[\reverb],
				\out, ~out,
			],
			~reverbGrp
		);

		~synth1 = Synth.new(\IMUSynth_1, target:~mainGrp);
		~d1_btn0_state = false;
		~d1_btn1_state = false;
		~d1_btn2_state = false;
		~d1_btn3_state = false;
		~d2_btn0_state = false;
		~d2_btn1_state = false;
		~d2_btn2_state = false;
		~d2_btn3_state = false;
	});
};

~makeEvents = {
	// MIDIIn.connectAll;
	e = Dictionary.new;

	e.add(\event1 -> {

		~drone = Pbind(
			\instrument, \bpfsaw,
			\dur, 1,
			\scale, Scale.minorPentatonic,
			\degree, Pwrand([-12,-9,-7,-6], [8,6,4,2].normalizeSum, inf),
			\detune, Pwhite(0.03,0.2),
			\rqmin, 0.08,
			\rqmax, 0.12,
			\cfmin, 50,
			\cfmax, 400,
			\atk, 2,
			\sus, 0.1,
			\rel, 2,
			\amp, 0.5,
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -20,
		).play;

		//infinite sequence of various finite rhythmic patterns
		//all very short envelopes
		Pdef(\rhythms,
			Pwrand([
				Pbind(
					\instrument, \osc,
					\dur,Pseq([1/8],4),
					\freq, Pstutter(4, Prand([
						Pexprand(10000,20000,1),
						Pexprand(100,200,1),
						Pexprand(1,2,1)
					],inf)),
					\detune, 100,
					\buf, Pstutter(4, Prand(~wt_buf[5..9],inf)),
					\atk, 0,
					\sus, 0,
					\rel, Pstutter(2, Pexprand(0.01,0.06)),
					\c1, exprand(8,20).neg,
					\amp, Pgeom(0.9, -6.dbamp, 4) * Pstutter(4,Pexprand(0.3,1)),
					\pan, Pwhite(-0.6,0.6),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-30,-15),
				),

				Pbind(
					\instrument, \osc,
					\dur, Pseq([1/4],2),
					\freq, Pstutter(2, Pexprand(1,200)),
					\detune, Pstutter(2, Pexprand(1,100)),
					\buf, Pstutter(2, Prand(~wt_buf[8..9],inf)),
					\atk, 0,
					\sus, 0,
					\rel, Pstutter(2, Pexprand(0.01,0.2)),
					\c1, -10,
					\amp, Pgeom(0.4, -3.dbamp, 2)  * Pexprand(0.4,1),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-30,-15),
				),

				Pbind(
					\instrument, \osc,
					\dur, Pseq([1/2,1/4,1/4],1),
					\freq, Pstutter(6, Pexprand(1000,2000)),
					\detune, 100,
					\buf, Pstutter(6, Prand(~wt_buf[2..5],inf)),
					\atk, 0,
					\sus, Pseq([1/3,0,0],1),
					\rel, Pseq([0,Pexprand(0.01,0.3,2)],1),
					\c1, -12,
					\amp, Pseq([0.1,0.5,0.3],1),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-30,-18),
				),

				Pbind(
					\instrument, \osc,
					\dur, Pseq([1/4,1/2,1/4],1),
					\freq, Pstutter(6, Pexprand(1000,2000)),
					\detune, 100,
					\buf, Pstutter(6, Prand(~wt_buf[2..5],inf)),
					\atk, 0,
					\sus, Pseq([0,1/3,0],1),
					\rel, Pseq([Pexprand(0.01,0.3,1),0,Pexprand(0.01,0.3,1)],1),
					\c1, -12,
					\amp, Pseq([0.5,0.1,0.4],1),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-30,-18),
				),

				Pbind(
					\instrument, \osc,
					\dur, Pseq([1/6],6),
					\freq, Pstutter(6, Pexprand(1,200)),
					\detune, Pstutter(6, Pexprand(1,100)),
					\buf, Pstutter(6, Prand(~wt_buf[8..9],inf)),
					\atk, 0,
					\sus, 0,
					\rel, Pstutter(6, Pexprand(0.01,0.1)),
					\c1, -10,
					\amp, Pgeom(0.7, -4.dbamp, 6)  * Pexprand(0.4,1),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-30,-18),
				),

				Pbind(
					\instrument, \osc,
					\dur, Prand([
						Pseq([1/2],2),
						Pseq([1],2),
						Pseq([1,1/2,1/2],1),
						Pseq([2],1),
					],1),
					\freq, Pstutter(2, Pexprand(1,200)),
					\detune, Pstutter(2, Pexprand(1,100)),
					\buf, Pstutter(2, Prand(~wt_buf[8..9],inf)),
					\atk, 0,
					\sus, 0,
					\rel, Pstutter(2, Pexprand(0.01,0.2)),
					\c1, -10,
					\amp, 0.5,
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, Pwhite(-20,-10),
				),

				Pbind(
					\instrument, \osc,
					\dur, Prand([
						Pseq([1/16],16),
						Pseq([1/16],8)
					],1),
					\freq, Pstutter(16,Pexprand(1000,20000,inf)),
					\detune, 0,
					\buf, Pstutter(16, Prand(~wt_buf[0..9],inf)),
					\atk, 0,
					\sus, 0,
					\rel, Pexprand(0.02,0.04),
					\c1, -4,
					\amp, 0.13,
					\pan, Pseq([1,-1],inf),
					\group, ~mainGrp,
					\out, ~out,
					\rout, ~bus[\reverb],
					\rsend, -30,
				)
			],
			[40,18,3,3,15,25,5].normalizeSum, inf)
		).play(t,quant:1);
	});

	e.add(\event2 -> {
		Pdef(\rhythms).stop;
		~drone.stop;
	});

	e.add(\event3 -> {

		~kick = Pbind(
			\instrument, \kick,
			\dur, Pseq([Pseq([1,1,1,1],3), Pseq([1,1,1,0.75,0.25],1)], inf),
			\freqA, 500,
			\atk, 0.01,
			\rel, 1,
			\amp, 0.8,
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -40,
		).play(t, quant:1);

	});

	e.add(\event4 -> {
		~kick.stop;
	});

	e.add(\event5 -> {

		~snare = Pbind(
			\instrument, \snare,
			\dur, Pwrand(
				[Pseq([Rest(1),1,Rest(1),1],1), Pseq([Rest(1),1,Rest(1),Rest(0.50),0.25,0.25],1)],
				[90,10].normalizeSum,
				inf),
			\atk, 0.01,
			\rel, 0.5,
			\c1, -10,
			\freqHPF, 300,
			\rq, 5,
			\amp, 0.8,
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -30
		).play(t,quant:1);

	});

	e.add(\event6 -> {
		~snare.stop;
	});

	e.add(\event7 -> {
		~pad.stop;

		~chords = Pbind(
			\instrument, \bpfsaw,
			\dur, Pwhite(4.5,7.0),
			\scale, Scale.minorPentatonic,
			\degree, Pxrand([
				[0,3,5],
				[1,2,4],
				[3,6,9] - 12,
				[3,5,7]
			], inf),
			\detune, Pexprand(0.05,0.2),
			\cfmin, 100,
			\cfmax, 1500,
			\rqmin, Pexprand(0.01,0.15),
			\atk, Pwhite(2.0,2.5),
			\rel, Pwhite(6.5,10.0),
			\ldb, 6,
			\amp, 0.2,
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -20,
		).play(t, quant:1);
	});

	e.add(\event8 -> {
		~chords.stop;
	});

	e.add(\event9 -> {
		~chords.stop;

		//background pad using simple wavetables
		~pad = Pbind(
			\instrument, \osc,
			\dur, Pwrand([1,4,6,9,12],[0.35,0.25,0.2,0.15,0.05],inf),
			\atk, Pexprand(3,6),
			\sus, 0,
			\rel, Pexprand(5,10),
			\c0, Pexprand(1,2),
			\c1, Pexprand(1,2).neg,
			\detune, Pfunc({rrand(0.15,0.4)}!3),
			\buf, Prand(~wt_buf[2..5], inf),
			\scale, Scale.minorPentatonic,
			\degree, Pfunc({
				(-12,-10..12).scramble[0..rrand(1,3)]
			}),
			\amp, Pexprand(0.05,0.07),
			\pan, Pwhite(-0.4,0.4),
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -10,
		).play;
	});

	e.add(\event10 -> {
		~pad.stop;
	});

	e.add(\event11 -> {

		~sawPtrn = Pbind(
			\instrument, \sawsynth,
			\dur, 1/4,
			\scale, Scale.minorPentatonic,
			\degree, Pseq([0,-1,-2,-3,-4,-5,-6,-7], inf),
			\amp, Pwrand([0.1,0.2], [8,1].normalizeSum, inf),
			\atk, 0.01,
			\rel, 0.3,
			\detune, 0.2,
			\buf, Pseq([Pseq(~wt_buf[0..7],1)], inf),
			\hrm, 4,
			\pan, Pwhite(-1.0,1.0),
			\group, ~mainGrp,
			\out, ~out,
			\rout, ~bus[\reverb],
			\rsend, -30
		).play(t, quant:1);

	});

	e.add(\event12 -> {
		~sawPtrn.stop;
	});

	e.add(\event13 -> {
		Pdef(\rhythms).stop;
		~drone.stop;
		~kick.stop;
		~snare.stop;
		~chords.stop;
		~pad.stop;
		~sawPtrn.stop;
	});

	/////////////////////////////////// Device 1 ///////////////////////////////////////
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

		case
		{(msg[5] != ~d1_btn1_state) && (msg[5]==true)}
		{e[\event1].value;"event1".postln; ~d1_btn1_state = msg[5];}

		{(msg[5] != ~d1_btn1_state) && (msg[5]==false)}
		{e[\event2].value;"event2".postln; ~d1_btn1_state = msg[5];}

		{(msg[6] != ~d1_btn2_state) && (msg[6]==true)}
		{e[\event3].value;"event3".postln; ~d1_btn2_state = msg[6];}

		{(msg[6] != ~d1_btn2_state) && (msg[6]==false)}
		{e[\event4].value;"event4".postln; ~d1_btn2_state = msg[6];}

		{(msg[7] != ~d1_btn3_state) && (msg[7]==true)}
		{e[\event5].value;"event5".postln; ~d1_btn3_state = msg[7];}

		{(msg[7] != ~d1_btn3_state) && (msg[7]==false)}
		{e[\event6].value;"event6".postln; ~d1_btn3_state = msg[7];}

		{true} {nil};

		// Post values
		// x = [msg[0], freq, cutoff, mod, gate];				// Assign freq, cutoff, and mod to array x
		// x.postln;						// Post [freq, cutoff, mod] values to window

		// Send values to synth object
		~synth1.set(\freq, freq, \lpf, cutoff, \fmHz, mod, \gate, gate);
	}, '/device1');


	/////////////////////////////////// Device 2 ///////////////////////////////////////
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

		case
		{(msg[4] != ~d2_btn1_state) && (msg[4]==true)}
		{e[\event7].value;"event7".postln; ~d2_btn1_state = msg[4];}

		{(msg[4] != ~d2_btn1_state) && (msg[4]==false)}
		{e[\event8].value;"event8".postln; ~d2_btn1_state = msg[4];}

		{(msg[5] != ~d2_btn2_state) && (msg[5]==true)}
		{e[\event9].value;"event9".postln; ~d2_btn2_state = msg[5];}

		{(msg[5] != ~d2_btn2_state) && (msg[5]==false)}
		{e[\event10].value;"event10".postln; ~d2_btn2_state = msg[5];}

		{(msg[6] != ~d2_btn3_state) && (msg[6]==true)}
		{e[\event11].value;"event11".postln; ~d2_btn3_state = msg[6];}

		{(msg[6] != ~d2_btn3_state) && (msg[6]==false)}
		{e[\event12].value;"event12".postln; ~d2_btn3_state = msg[6];}

		{(msg[7] != ~d2_btn4_state) && (msg[7]==true)}
		{e[\event13].value;"event13".postln; ~d2_btn4_state = msg[7];}

		{(msg[7] != ~d2_btn4_state) && (msg[7]==false)}
		{
			// e[\event14].value;"event14".postln;
			~d2_btn4_state = msg[7];
		}

		{true} {nil};

		// Post values
		// x = [msg[0], amp, mod2, pan];				// Assign amp, mod2, and pan to array x
		// x.postln;						// Post [amp, mod2, pan] values to window

		// Send values to synth object
		~synth1.set(\amp, amp, \fmAmp, mod2, \pan, pan);
	}, '/device2');

/*	OSCdef(\d2_buttons, {
		arg msg;
		msg.postln;
		case
		{msg[4]==0} {e[\event1].value;"event1".postln;}
		{msg[4]==1} {e[\event2].value;"event2".postln;}
		{msg[5]==0} {e[\event3].value;"event3".postln;}
		{msg[5]==1} {e[\event4].value;"event4".postln;}
		{msg[6]==0} {e[\event5].value;"event5".postln;}
		{msg[6]==1} {e[\event6].value;"event6".postln;}
		{msg[7]==0} {e[\event7].value;"event7".postln;}
		{msg[7]==1} {e[\event8].value;"event8".postln;}
		// {msg[1]==9} {e[\event9].value;"event9".postln;}
		// {msg[1]==10} {e[\event10].value;"event10".postln;}
		// {msg[1]==11} {e[\oneshot1].value;"oneshot1".postln;}
		{true} {nil};
	}, '/device2');*/
};

//4. register functions with ServerBoot/Quit/Tree
ServerBoot.add(~makeBuffers);
ServerBoot.add(~makeBusses);
ServerQuit.add(~cleanup);

//5. boot server
s.waitForBoot({

	s.sync;

	//6a. SynthDefs
	SynthDef.new(\IMUSynth_1,{
		arg freq=600.0, detune=0.2, amp=0.2, pan=0.0, gate=0,
		fmHz=200.0, fmAmp=0.0, lpf=15000,
		atk=0.01, rel=1.0, curv=(-4.0),
		out=~out, rout=~bus[\reverb], rsend=(-20);
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
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp); //"post-fader" send to reverb
	}).add;

	SynthDef(\bpfsaw, {
		arg atk=2, sus=0, rel=3, c1=1, c2=(-1),
		freq=500, detune=0.2, pan=0, cfhzmin=0.1, cfhzmax=0.3,
		cfmin=500, cfmax=2000, rqmin=0.1, rqmax=0.2,
		lsf=200, ldb=0, amp=1, out=0, rout=0, rsend=(-20);
		var sig, env;
		env = EnvGen.kr(Env([0,1,1,0],[atk,sus,rel],[c1,0,c2]),doneAction:2);
		sig = Saw.ar(freq * {LFNoise1.kr(0.5,detune).midiratio}!2);
		sig = BPF.ar(
			sig,
			{LFNoise1.kr(
				LFNoise1.kr(4).exprange(cfhzmin,cfhzmax)
			).exprange(cfmin,cfmax)}!2,
			{LFNoise1.kr(0.1).exprange(rqmin,rqmax)}!2
		);
		sig = BLowShelf.ar(sig, lsf, 0.5, ldb);
		sig = Balance2.ar(sig[0], sig[1], pan);
		sig = sig * env * amp;
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp); //"post-fader" send to reverb
	}).add;

	SynthDef(\osc, {
		arg buf=0, freq=200, detune=0.2,
		amp=0.2, pan=0, out=0, rout=0, rsend=(-20),
		freqfilt = 15000, hrm=4, rq=0.5,
		atk=0.01, sus=1, rel=0.01, c0=1, c1=(-1);
		var sig, env, detuneCtrl;
		env = EnvGen.ar(
			Env([0,1,1,0],[atk,sus,rel],[c0,0,c1]),
			doneAction:2
		);

		//array of eight Oscs with uniquely detune frequencies
		//and unique initial phase offsets
		detuneCtrl = LFNoise1.kr(0.1!8).bipolar(detune).midiratio;
		sig = Osc.ar(buf, freq * detuneCtrl, {Rand(0,2pi)}!8);

		sig = Splay.ar(sig); //spread 8 signals over stereo field
		sig = LeakDC.ar(sig); //remove DC bias
		sig = Balance2.ar(sig[0], sig[1], pan, amp); //L/R balance (pan)
		sig = sig * env;
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp); //"post-fader" send to reverb
	}).add;

	SynthDef(\sawsynth,
	{
		arg freq=400, detune=0.2, atk=0.01, rel=1,
			filtfreq=200, hrm=4, rq=0.5,
			amp=0.2, pan=0, out=0, rout=0, rsend=(-20);
		var sig, env;
		env = Env.perc(atk, rel).kr(2);
		sig = Saw.ar([freq, freq * detune.midiratio]);
		sig = RLPF.ar(sig, (freq * hrm).clip(20, 20000), 0.5);
		sig = sig * env * amp;
		sig = Pan2.ar(sig,pan);
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp);
	}).add;

	SynthDef(\kick, {
		arg freqA=500, freqB=50, freqC=10, freqDur1=0.01, freqDur2=0.2, freqC1=1, freqC2=(-1),
		atk=0.01, rel=1, c1=1, c2=(-12), amp=0.5, pan=0, out=0, rout=0, rsend=(-20);
		var sig, env, freqSweep;
		freqSweep = Env([freqA, freqB, freqC], [freqDur1, freqDur2], [freqC1, freqC2]).ar;
		env = Env([0,1,0], [atk,rel], [c1, c2]).kr(2);
		sig = SinOsc. ar(freqSweep, pi/2);
		sig = sig * env;
		sig = Pan2.ar(sig, pan, amp);
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp); //"post-fader" send to reverb
	}).add;

	SynthDef(\snare, {
		arg atk=0.01, rel=1.0, c1=(-12), freqHPF=440.0, rq=1.0,
		amp=0.5, pan=0.0, out=0, rout=0, rsend=(-20);
		var sig, env;
		env = Env.perc(atk, rel, curve:c1).kr(2);
		sig = RHPF.ar(PinkNoise.ar(), freqHPF, rq);
		sig = sig * env;
		sig = Pan2.ar(sig, pan, amp);
		Out.ar(out, sig);
		Out.ar(rout, sig * rsend.dbamp); //"post-fader" send to reverb
	}).add;

	SynthDef(\reverb, {
		arg in=0, out=0, predelay=0.1, revtime=4, lpf=4500;
		var sig;
		sig = In.ar(in, 2).sum;
		sig = DelayN.ar(sig, 0.2, predelay);
		// sig = CombN.ar(sig, 0.1, {Rand(0.01,0.099)}!32, revtime);
		// sig = SplayAz.ar(2, sig);
		// sig = LPF.ar(sig, lpf);
		16.do{sig = AllpassN.ar(sig, 0.05, {Rand(0.001,0.05)}!2, revtime)};
		sig = LPF.ar(sig, lpf);
		sig = LeakDC.ar(sig);
		Out.ar(out, sig);
	}).add;

	s.sync;

	//6b. register remaining functions
	ServerTree.add(~makeNodes);
	ServerTree.add(~makeEvents);
	s.freeAll;

	s.sync;

	s.meter;							// Open level meter
	s.plotTree;							// Open Node Tree

	s.sync;

	"done".postln;
}, 5, "boot fail".postln);
)

NetAddr.localAddr    // retrieve the current IP and port

n = NetAddr.new("127.0.0.1", 57120);
n.sendMsg('/control', 10);

OSCdef.freeAll        // Free all OSC objects
s.quit;


