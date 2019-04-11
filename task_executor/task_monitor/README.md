# Assistance Arbitrator

Given an incoming assistance goal, the arbitrator uses a method selected through config files to decide how to resolve the request. When it picks a method of resolution, called a **strategy**, it then forwards the request on to the appropriate strategy.

There are two strategies:

1. [Local Strategy](local_interfaces/local_strategy/): The robot looks for a local human, approaches them, and tries to solicit help from this human.
1. [Remote Strategy](remote_interfaces/remote_strategy): The robot sends a request to a remote interface, which is then presented to a user on that interface.


## Notes

- Probably should have used the diagnostics topic from the start to keep a trace of the program execution. Too late to start now.
- Instead, make sure to bag the trace topic and the diagnostics topic; we'll use both to create a complete trace of execution post-facto.
