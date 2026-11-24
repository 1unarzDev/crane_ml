#!/usr/bin/env python3
"""Inspect CRANE authoritative replay streams without loading Unity."""

import argparse
import hashlib
import json
import os
import struct
from collections import Counter


STREAM_MAGIC = 0x4352414E
INDEX_MAGIC = 0x43524958
FRAME_MAGIC = 0x4652414D
INDEX_ENTRY_BYTES = 25


def read_exact(stream, size):
    value = stream.read(size)
    if len(value) != size:
        raise EOFError(f"expected {size} bytes, received {len(value)}")
    return value


def unpack(stream, fmt):
    size = struct.calcsize(fmt)
    return struct.unpack(fmt, read_exact(stream, size))


def read_7bit_int(stream):
    value = 0
    shift = 0
    while shift < 35:
        byte = read_exact(stream, 1)[0]
        value |= (byte & 0x7F) << shift
        if byte & 0x80 == 0:
            return value
        shift += 7
    raise ValueError("invalid BinaryWriter string length")


def read_string(stream):
    size = read_7bit_int(stream)
    return read_exact(stream, size).decode("utf-8")


def skip_strings(stream):
    count, = unpack(stream, "<i")
    if count < 0:
        raise ValueError(f"invalid string count {count}")
    for _ in range(count):
        read_string(stream)


def skip_bodies(stream):
    count, = unpack(stream, "<i")
    if count < 0:
        raise ValueError(f"invalid body count {count}")
    for _ in range(count):
        read_string(stream)
        read_exact(stream, 1 + 12 + 16 + 12 + 12)
        joint_count, = unpack(stream, "<i")
        if joint_count < 0:
            raise ValueError(f"invalid joint count {joint_count}")
        read_exact(stream, joint_count * 4)


def read_water(stream):
    count, = unpack(stream, "<i")
    if count < 0:
        raise ValueError(f"invalid water count {count}")
    states = []
    for _ in range(count):
        surface = read_string(stream)
        simulation_time, time_multiplier = unpack(stream, "<ff")
        states.append({
            "surface": surface,
            "simulationTime": simulation_time,
            "timeMultiplier": time_multiplier,
        })
    return states


def inspect(path, max_actions, max_outcomes, max_observations):
    actions = []
    outcomes = []
    observations = []
    sources = Counter()
    encodings = Counter()
    outcome_sources = Counter()
    action_count = 0
    outcome_count = 0
    observation_count = 0
    observation_topics = Counter()
    observation_types = Counter()
    observation_payload_bytes = 0
    terminated_count = 0
    truncated_count = 0
    frame_count = 0
    discontinuities = 0
    first_tick = None
    last_tick = None
    first_metric_tick = None
    last_metric_tick = None
    metric_tick_regressions = 0
    metric_tick_resets = 0
    last_water = []

    with open(path, "rb") as stream:
        magic, version = unpack(stream, "<ii")
        if magic != STREAM_MAGIC or version not in (1, 2, 3, 4, 5):
            raise ValueError(f"unsupported CRANE stream magic/version: {magic:#x}/{version}")
        unity_version = read_string(stream)
        build_guid = read_string(stream)
        scene = read_string(stream)
        configuration_hash = read_string(stream)
        fixed_delta, utc_start_ticks = unpack(stream, "<fq")
        build_manifest_hash = None
        build_manifest = None
        runtime_metadata = None
        build_manifest_hash_valid = None
        if version >= 4:
            build_manifest_hash = read_string(stream)
            build_manifest_json = read_string(stream)
            runtime_metadata_json = read_string(stream)
            computed_hash = (hashlib.sha256(build_manifest_json.encode("utf-8"))
                             .hexdigest().upper() if build_manifest_json else "")
            build_manifest_hash_valid = computed_hash == build_manifest_hash.upper()
            build_manifest = json.loads(build_manifest_json) if build_manifest_json else None
            runtime_metadata = json.loads(runtime_metadata_json) if runtime_metadata_json else None

        while stream.tell() < os.path.getsize(path):
            frame_magic, = unpack(stream, "<i")
            if frame_magic != FRAME_MAGIC:
                raise ValueError(f"invalid frame magic at offset {stream.tell() - 4}")
            tick, metric_tick, simulation_time, episode_id = unpack(stream, "<qqdq")
            discontinuity = read_exact(stream, 1)[0] != 0
            frame_count += 1
            discontinuities += int(discontinuity)
            first_tick = tick if first_tick is None else first_tick
            last_tick = tick
            first_metric_tick = metric_tick if first_metric_tick is None else first_metric_tick
            if last_metric_tick is not None and metric_tick < last_metric_tick:
                if discontinuity:
                    metric_tick_resets += 1
                else:
                    metric_tick_regressions += 1
            last_metric_tick = metric_tick
            skip_strings(stream)
            skip_strings(stream)
            skip_bodies(stream)
            last_water = read_water(stream)

            if version < 2:
                continue
            frame_action_count, = unpack(stream, "<i")
            if frame_action_count < 0:
                raise ValueError(f"invalid action count {frame_action_count}")
            for _ in range(frame_action_count):
                source = read_string(stream)
                action_episode, sequence, source_tick, receive_tick, application_tick = unpack(
                    stream, "<qqqqq")
                encoding = read_string(stream)
                payload_size, = unpack(stream, "<i")
                if payload_size < 0 or payload_size > 64 * 1024 * 1024:
                    raise ValueError(f"invalid action payload size {payload_size}")
                payload = read_exact(stream, payload_size)
                action_count += 1
                sources[source] += 1
                encodings[encoding] += 1
                if len(actions) < max_actions:
                    actions.append({
                        "frameTick": tick,
                        "metricTick": metric_tick,
                        "simulationTime": simulation_time,
                        "source": source,
                        "episodeId": action_episode,
                        "sequence": sequence,
                        "sourceObservationTick": source_tick,
                        "receiveTick": receive_tick,
                        "applicationTick": application_tick,
                        "payloadEncoding": encoding,
                        "payloadBytes": payload_size,
                        "payloadHex": payload.hex(),
                    })

            if version < 3:
                continue
            frame_outcome_count, = unpack(stream, "<i")
            if frame_outcome_count < 0:
                raise ValueError(f"invalid outcome count {frame_outcome_count}")
            for _ in range(frame_outcome_count):
                source = read_string(stream)
                outcome_episode, sequence, outcome_tick = unpack(stream, "<qqq")
                reward, cumulative_reward = unpack(stream, "<dd")
                terminated = read_exact(stream, 1)[0] != 0
                truncated = read_exact(stream, 1)[0] != 0
                reason = read_string(stream)
                outcome_count += 1
                terminated_count += int(terminated)
                truncated_count += int(truncated)
                outcome_sources[source] += 1
                if len(outcomes) < max_outcomes:
                    outcomes.append({
                        "frameTick": last_tick,
                        "source": source,
                        "episodeId": outcome_episode,
                        "sequence": sequence,
                        "tick": outcome_tick,
                        "reward": reward,
                        "cumulativeReward": cumulative_reward,
                        "terminated": terminated,
                        "truncated": truncated,
                        "reason": reason,
                    })

            if version < 5:
                continue
            frame_observation_count, = unpack(stream, "<i")
            if frame_observation_count < 0:
                raise ValueError(f"invalid observation count {frame_observation_count}")
            for _ in range(frame_observation_count):
                source = read_string(stream)
                topic = read_string(stream)
                frame_id = read_string(stream)
                message_type = read_string(stream)
                encoding = read_string(stream)
                payload_policy = read_string(stream)
                (observation_episode, sequence, acquisition_tick, acquisition_time,
                 completion_tick, completion_time, width, height, row_step, element_count,
                 payload_bytes) = unpack(stream, "<qqqdqdiiiiq")
                observation_count += 1
                observation_topics[topic] += 1
                observation_types[message_type] += 1
                observation_payload_bytes += payload_bytes
                if len(observations) < max_observations:
                    observations.append({
                        "frameTick": tick,
                        "source": source,
                        "topic": topic,
                        "frameId": frame_id,
                        "messageType": message_type,
                        "encoding": encoding,
                        "payloadPolicy": payload_policy,
                        "episodeId": observation_episode,
                        "sequence": sequence,
                        "acquisitionTick": acquisition_tick,
                        "acquisitionTime": acquisition_time,
                        "completionTick": completion_tick,
                        "completionTime": completion_time,
                        "latencyTicks": completion_tick - acquisition_tick,
                        "width": width,
                        "height": height,
                        "rowStep": row_step,
                        "elementCount": element_count,
                        "payloadBytes": payload_bytes,
                    })

    index_path = path + ".index"
    index_entries = None
    index_version = None
    if os.path.isfile(index_path):
        with open(index_path, "rb") as index:
            index_magic, index_version = unpack(index, "<ii")
            if index_magic != INDEX_MAGIC:
                raise ValueError(f"unsupported CRANE index magic {index_magic:#x}")
            remaining = os.path.getsize(index_path) - 8
            if remaining < 0 or remaining % INDEX_ENTRY_BYTES != 0:
                raise ValueError("invalid CRANE index length")
            index_entries = remaining // INDEX_ENTRY_BYTES

    return {
        "schema": "crane-replay-inspection-v1",
        "path": os.path.abspath(path),
        "version": version,
        "unityVersion": unity_version,
        "buildGuid": build_guid,
        "scene": scene,
        "configurationHash": configuration_hash,
        "buildManifestHash": build_manifest_hash,
        "buildManifestHashValid": build_manifest_hash_valid,
        "buildManifest": build_manifest,
        "runtimeMetadata": runtime_metadata,
        "fixedDeltaTime": fixed_delta,
        "utcStartTicks": utc_start_ticks,
        "frameCount": frame_count,
        "firstTick": first_tick,
        "lastTick": last_tick,
        "firstMetricTick": first_metric_tick,
        "lastMetricTick": last_metric_tick,
        "metricTickRegressions": metric_tick_regressions,
        "metricTickResets": metric_tick_resets,
        "discontinuities": discontinuities,
        "lastWater": last_water,
        "indexVersion": index_version,
        "indexEntries": index_entries,
        "indexMatchesFrames": index_entries == frame_count if index_entries is not None else None,
        "actionCount": action_count,
        "actionSources": dict(sorted(sources.items())),
        "actionEncodings": dict(sorted(encodings.items())),
        "actionsTruncated": action_count > len(actions),
        "actions": actions,
        "outcomeCount": outcome_count,
        "outcomeSources": dict(sorted(outcome_sources.items())),
        "terminatedCount": terminated_count,
        "truncatedCount": truncated_count,
        "outcomesTruncated": outcome_count > len(outcomes),
        "outcomes": outcomes,
        "observationCount": observation_count,
        "observationTopics": dict(sorted(observation_topics.items())),
        "observationTypes": dict(sorted(observation_types.items())),
        "observationPayloadBytes": observation_payload_bytes,
        "observationsTruncated": observation_count > len(observations),
        "observations": observations,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("replay")
    parser.add_argument("--max-actions", type=int, default=1000)
    parser.add_argument("--max-outcomes", type=int, default=1000)
    parser.add_argument("--max-observations", type=int, default=1000)
    parser.add_argument("--output")
    args = parser.parse_args()
    result = inspect(args.replay, max(0, args.max_actions), max(0, args.max_outcomes),
                     max(0, args.max_observations))
    rendered = json.dumps(result, indent=2) + "\n"
    if args.output:
        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as output:
            output.write(rendered)
    else:
        print(rendered, end="")


if __name__ == "__main__":
    main()
