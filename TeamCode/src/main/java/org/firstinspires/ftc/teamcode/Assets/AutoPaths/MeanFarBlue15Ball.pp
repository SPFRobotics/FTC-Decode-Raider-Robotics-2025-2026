{
  "version": "1.7.5",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 56,
    "y": 9,
    "heading": "linear",
    "startDeg": 180,
    "endDeg": -180,
    "locked": false
  },
  "lines": [
    {
      "id": "line-xhmxzrfd0j8",
      "name": "",
      "endPoint": {
        "x": 41.16605166051661,
        "y": 36,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [],
      "color": "#56DD68",
      "eventMarkers": [],
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmc7anyf-v2vk1w",
      "name": "",
      "endPoint": {
        "x": 24.07749077490775,
        "y": 36,
        "heading": "tangential",
        "reverse": false
      },
      "controlPoints": [],
      "color": "#7AC87A",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-xhmxzrfd0j8"
    },
    {
      "kind": "path",
      "lineId": "mmc7anyf-v2vk1w"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 144,
          "y": 69.5
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 119,
          "y": 144
        },
        {
          "x": 137.5,
          "y": 119
        },
        {
          "x": 137.5,
          "y": 69.5
        }
      ],
      "color": "#dc2626",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 6.5,
          "y": 119
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 0,
          "y": 69.5
        },
        {
          "x": 6.5,
          "y": 69.5
        }
      ],
      "color": "#0b08d9",
      "fillColor": "#fca5a5",
      "type": "obstacle"
    }
  ],
  "extraData": {}
}