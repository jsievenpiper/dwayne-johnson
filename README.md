# Dwayne "Robo Rock" Johnson
## it's a rock, get it?

The goal of this project is to build a low-profile 4'x4' frame, which is driven by a pair of 24V 200W continuous / 1200W
peak DC motors. This then becomes the basis of an automatable wagon of sorts that can be used for a variety of use
cases. These motors are mounted parallel to each other on the center of one axis such that the wagon is capable of
turning with zero-radius, allowing for in-place rotation. This frame is then intended to be controlled via human input
"remotely" at a distance off-stage. To do so, we're using a PlayStation DualShock 4 controller, but really any
controller with a minimum of two linear/analog inputs would work. The DualShock 4 is convenient because it has a bunch
of these types of inputs from the analog sticks themselves to the shoulder triggers.

From an electronics perspective, each motor is driven with an independent motor driver that takes a 10-20kHz PWM signal
to control the motor speed and a digital directional pin that can pulled high or low to change the polarity of the motor
and therefore the direction of the motor's rotation. These motor drivers are implemented as relatively straightforward
H-bridges composed of N-type MOSFETs.

To generate these PWM and digital control signals and receive controller input, the heart of the system is powered by an
ESP32 (other variants of the ESP32 like the C3 etc _won't_ work with the DualShock series due to needing "Classic"
non-BLE -- but you may find a controller that supports BLE in which case you can be on your merry way). I've "enhanced"
the ESP32 a bit to fit a larger antenna to hopefully increase and stabilize our Bluetooth range. We'll see how it pans
out.

In the last project where we drove these motors, an Orange Pi/Raspberry Pi/GPIO-laden SBC was used to drive the same
motor controllers. In that scenario, the motor drive was automated into lighting cues via DMX (ArtNet/OSC/sACN) and was
for a non-mission-critical prop, so things like response latency where less critical than network reliability (if the 
motor took 100ms to respond or was preempted by another OS-level task, no big deal -- humanly inperceptible). But now,
this real-time input that will navigate among performers on stage (and carry some!) requires more predictability and
guarantees than the DMX-driven system. Going direct to the microcontroller with a real-time operating system adds that
reliability and simplicity that keeps confidence levels high. It also doesn't hurt that the ESP32 is like, five bucks.

Instead of an ESP32, I _think_ an RP2040 would work, which are also pretty readily available. You'd want a board that
also pairs with a 2.4GHz radio for Bluetooth support. I think the Pico W fits the bill (and supports Bluetooth classic),
so that can also be a pretty reasonable alternative. The RP2040 is a fair bit easier to flash, in my opinion, if that
carries any weight.

The ESP32 has two complete cores that can be driven in parallel. In our use case, due the weight of a Bluetooth stack,
we dedicate one core to reliably servicing that side of the problem by continously reading inputs from the controller
and converting them into an intermediate state structure. This state structure is incredibly simple: it simply describes
what our rock might be doing at any instantaneous point in time. This really boils down to two (well, four) things: how
quickly we want the motors to be turning, and in what direction. These states are continuously collected by the
Bluetooth side of our processing loop and passed along to the other core via a message buffer implemented in shared
memory.

![An image of the core utilization architecture](/cores.png)

On the other side of the chip, we regularly check the message buffer for any new "desired" states. I emphasize
_desired_, because the goal of this side of the chip is to take these states and mesh them with reality. We do this by
actually keeping track of _two_ states: the latest of the "desired" states, and another copy of that exact state
representation for the _current_ state. For example, when Dwayne first starts up, the _current state_ of the motors is
to be stopped -- a speed magnitude of zero and going neither forward nor backward. At that point, the operator may
decide to suddenly jam one or both of the controller sticks in a direction. The rock cannot instantaneously be going at
full speed, nor would a human probably want that to happen (these motors are pretty darn torquey... so while they WILL
whip you around pretty aggressively, physics is still a thing). Similarly if the rock is already moving a direction and
the operator suddenly changes direction, we try to treat our rock nicely and not try to spin it backwards while moving.

So what do we do? Well, each iteration of this side of the chip looks something like this:

![An image of the full architecture diagram](/full-diagram.png)

Avid software afficionados will recognize this as deeply inspired by The Elm Architecture, and that's because... well,
it is! The Elm Architecture, applied to a giant robotic rock, works like so. First, we use the model of the _current_
state of the system (this is the same state we talked about earlier that starts at an Neutral, zero-speed idle). This
current state is what is made real for each "tick" of our processor core. Zero speed, stopped rock, great. At the
beginning of each tick, we read off the aforementioned message queue to see what the latest _desired_ state is. In that
tick, we then mutate the current state in order to bring it more in line with what the desired state is. From a stand
still, that will be to accelerate one or both of the motors at some point, but once the motors are going, it could also
be to _decelerate_ the motors (either to bring them to a stop or just to slow down).

This is where the whole "desired versus reality" thing kicks in: instead of simply asking the motors to do _exactly_
what the operator is asking, we tween into that state over time. For example, instead of allowing for a jump from zero
to full speed, for each tick, we may only update the current state by some fixed amount headed in that direction. It
may take several tens or hundreds of ticks to catch the desired speed (remember this is computer world so even on a
cheap processor like this, we're 'thinking' at 240MhZ -- that's 240 _million_ calculations **per second**... per core.
We don't actually process controller input at that rate, because that would be insanely ridiculous (and it takes
thousands of calculations to calculate a "tick" of the system -- but suffice to say, it's still happening very quickly
and well beyond human reaction time and perception).

For both acceleration and deceleration, uniquely, we follow this pattern of "chasing" the system's desired state. In
some ticks we may be at the desired state, in which case there's nothing to do. In some scenarios the operator may be
constantly changing inputs and we may never actually hit the desired state. This process of chasing the desired state
while following these acceleration and deceleration profiles looks like this, visually:

![An image of how the system responds to inputs over time](/curves.png)

After the tweening is done, at the end of each tick we "render" out what the current state of the system should be at
that moment. This likely involves adjusting the duty cycle of the motor controllers and  potentially also changing the
polarity of the motor's supplied voltage via the H-Bridge inputs (this is only ever done when the motors are at a
standstill or going _extremely_ slowly). Rinse and repeat, for forever.

There's one last little addition that is incredibly simple to reason about: a max-speed controller. From experience, I
have very nearly sent prop pieces flying through my garage as spinning blades of death. Again, these motors are pretty
darn strong. A simple potentiometer was added to run through an ADC -- we don't actually care about the measured
voltage, but instead are just looking for some linear range. When calculating the duty cycle of the motors, the system's
calculated desired speed is then scaled by the ratio of how far the potentiometer has been dialed. That is, if the dial
is set to 30% (conveniently about how fast we run this thing in production -- for fun antics it should always be dialed
to the max), then the controller inputs map from 0->30% duty cycle rather than 0-100% -- effectively making the
controller less sensitive in the process, making tiny speed changes easier to manage and rendering the whole prop less
"twitchy".

Like all projects, I hope this inspires someone out there to build something awesome. I'd love to hear what you make or
help you make the next great robo rock! Break a leg!
