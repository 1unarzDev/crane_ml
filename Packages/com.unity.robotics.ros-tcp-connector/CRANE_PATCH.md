# CRANE ROS-TCP connector patch

This embedded package is based on Unity ROS-TCP Connector `0.7.0-preview` at Git commit
`c27f00c6cf750d2d0564349b3039d19aa3925e7c` (Unity package-cache fingerprint
`bdad6c87bd9e4952606b944d8c8757d08afeec7d`).

CRANE embeds it because the upstream connection path can register a publisher twice: a
registration queued before the TCP handshake is sent again immediately by
`RosTopicState.OnConnectionEstablished`. Topic creation may also race the connection thread's
dictionary enumeration.

The local patch:

- snapshots the topic dictionary under its existing lock;
- synchronizes get-or-create with that snapshot;
- tracks whether publisher registration is already queued/sent for the current connection;
- clears the registration state on disconnect so reconnect still registers exactly once; and
- serializes publisher state transitions against connection callbacks.

Keep this package pinned until an upstream release contains equivalent behavior and passes
CRANE's live ROS transport and reconnect validation.
