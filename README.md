# ai2018

Strategy from [Russian AI Cup 2018](http://russianaicup.ru)  
Strategy source itself is just [**MyStrategy.cpp**](MyStrategy.cpp) and [**MyStrategy.h**](MyStrategy.h), the rest is default c++17 package provided by orgs.

### Revisions

1. "Dumb strategy 1" - simple hit&run logic. Still doing great in beta stage though (: It even got to place 245 in round 1!
2. "L1 strategy" - ball trajectory prediction, nice defense but still miles away from top-10. Got to place 128 in round 2.
<br>
Lessons learned:
* You **really** need your own debug simulation. Local runner is good for testing but not suitable for debugging.
* Accounting for opponent bots is important. The L1 strategy could use 300+ ticks of ball prediction, but in reality if you do not hit the ball in 100, it is highly likely that opponent will. And I really regret not having enough time for creating a way to account for this.
