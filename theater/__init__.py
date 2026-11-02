"""crane ghost theater: pose playback of many recorded runs as one deterministic video.

``pack`` writes the render-ready track pack and manifest the Unity theater reads, ``synth`` makes
synthetic fleets for tests and spikes, ``worlds`` holds the domain plug-ins (course centre line,
bounds, player arguments), ``render`` runs a render locally and assembles chunk videos. Nothing in
``pack`` or ``render`` names a domain; a world is chosen by the manifest's ``world.plugin``.
"""
