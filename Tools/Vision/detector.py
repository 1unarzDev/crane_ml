# SPDX-License-Identifier: GPL-3.0-or-later
"""Onboard person detector used by the CRANE vision benchmark: RT-DETR-R18 (Apache-2.0), COCO-pretrained, class 'person'.

Weights: PekingU/rtdetr_r18vd (Hugging Face, Apache-2.0, sha ac77a11f). Loaded from a local directory (untrusted
download kept under ~/.cache/research/vision-data/weights_rtdetr_r18vd/); never committed.

Two inference modes on the 1920x1080 onboard RGB frame (RenderProfiles.OakD RGB stream):
  * 'full'  : letterbox the frame to 640x640 (image content 640x360): one forward pass, ~3x downscale.
  * 'tiled' : six 640x640 native-resolution tiles (x0 = 0, 640, 1280; y0 = 0, 440), batched; boxes merged by NMS.
Pre/post-processing is done on the GPU with plain torch ops (no HF image processor), so latency is end to end.
"""
import os

import numpy as np
import torch
import torchvision

DEFAULT_WEIGHTS = os.path.expanduser('~/.cache/research/vision-data/weights_rtdetr_r18vd')
PERSON = 0
TILE = 640
TILE_X = (0, 640, 1280)
TILE_Y = (0, 440)


class Detector:
    def __init__(self, weights=DEFAULT_WEIGHTS, device='cuda', half=False, score_floor=0.05):
        from transformers import RTDetrForObjectDetection
        self.device = torch.device(device)
        self.model = RTDetrForObjectDetection.from_pretrained(weights, use_safetensors=True).to(self.device).eval()
        self.half = half
        if half:
            self.model.half()
        self.score_floor = score_floor

    @torch.inference_mode()
    def _forward(self, batch):
        """batch: float tensor N x 3 x 640 x 640 in [0, 1]. Returns list of (boxes_xyxy_px, scores) for class person."""
        if self.half:
            batch = batch.half()
        out = self.model(pixel_values=batch)
        logits = out.logits.float()                     # N x Q x 80 (sigmoid focal)
        boxes = out.pred_boxes.float()                  # N x Q x 4 cx,cy,w,h normalised
        scores = logits[..., PERSON].sigmoid()
        res = []
        for b in range(batch.shape[0]):
            s = scores[b]
            k = s >= self.score_floor
            bx = boxes[b][k]
            xyxy = torch.stack([bx[:, 0] - bx[:, 2] / 2, bx[:, 1] - bx[:, 3] / 2,
                                bx[:, 0] + bx[:, 2] / 2, bx[:, 1] + bx[:, 3] / 2], 1) * TILE
            res.append((xyxy, s[k]))
        return res

    def to_tensor(self, img_rgb_u8):
        """HxWx3 uint8 numpy (RGB) -> 3xHxW float on device in [0,1]."""
        t = torch.from_numpy(np.ascontiguousarray(img_rgb_u8)).to(self.device, non_blocking=True)
        return t.permute(2, 0, 1).float().div_(255.0)

    @torch.inference_mode()
    def detect(self, img_rgb_u8, mode='tiled', nms_iou=0.5):
        """Returns (N x 4 xyxy boxes in frame pixels, N scores) as numpy, person class only."""
        x = self.to_tensor(img_rgb_u8)
        H, W = x.shape[1:]
        if mode == 'full':
            s = TILE / max(H, W)
            h, w = int(round(H * s)), int(round(W * s))
            r = torch.nn.functional.interpolate(x[None], size=(h, w), mode='bilinear', antialias=True, align_corners=False)
            canvas = torch.full((1, 3, TILE, TILE), 0.5, device=self.device)
            canvas[:, :, :h, :w] = r
            (b, sc), = self._forward(canvas)
            b = b / s
        elif mode == 'tiled':
            assert (H, W) == (1080, 1920), 'tiled mode expects the 1920x1080 onboard frame'
            tiles, offs = [], []
            for y0 in TILE_Y:
                for x0 in TILE_X:
                    tiles.append(x[:, y0:y0 + TILE, x0:x0 + TILE]); offs.append((x0, y0))
            outs = self._forward(torch.stack(tiles))
            bs, ss = [], []
            for (bb, s_), (x0, y0) in zip(outs, offs):
                # drop boxes touching an interior tile edge that another tile covers (split persons), keep the rest
                bs.append(bb + torch.tensor([x0, y0, x0, y0], device=self.device, dtype=bb.dtype)); ss.append(s_)
            b, sc = torch.cat(bs), torch.cat(ss)
            if len(sc):
                keep = torchvision.ops.nms(b, sc, nms_iou)
                b, sc = b[keep], sc[keep]
        else:
            raise ValueError(mode)
        return b.cpu().numpy(), sc.cpu().numpy()
