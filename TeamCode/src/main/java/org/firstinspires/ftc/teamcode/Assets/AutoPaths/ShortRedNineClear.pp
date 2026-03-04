{
  "version": "1.7.5",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 110.6,
    "y": 133.9,
    "heading": "linear",
    "startDeg": 0,
    "endDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-071zielfzy93",
      "name": "RunToShootPreload",
      "endPoint": {
        "x": 96.22386223862239,
        "y": 95.8228782287823,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#D57C7D",
      "eventMarkers": [],
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmbhbpee-uqd8np",
      "endPoint": {
        "x": 96.22386223862239,
        "y": 60.16359163591635,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#977ABC",
      "name": "RunToSpikeTwo",
      "eventMarkers": [],
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmbhc39j-jcmfha",
      "name": "IntakeSpikeTwo",
      "endPoint": {
        "x": 117.73800738007381,
        "y": 60.16359163591635,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#7BB5AB",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhcegu-x06y2x",
      "name": "RunToShootSpikeTwo",
      "endPoint": {
        "x": 94.9360393603936,
        "y": 95.8228782287823,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#AD89CD",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhekkz-21v0fh",
      "name": "RunToSpikeOne",
      "endPoint": {
        "x": 110.6,
        "y": 84.10824108241081,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#B9D77B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhf6xa-dlrsnu",
      "name": "IntakeSpikeOne",
      "endPoint": {
        "x": 118.91389913899138,
        "y": 84.10824108241081,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#5685C9",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhhwy8-vxt762",
      "name": "ClearRamp",
      "endPoint": {
        "x": 127.01107011070108,
        "y": 77.15375153751536,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#59D67B",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhi8oo-2twafu",
      "name": "RunToShootSpikeOne",
      "endPoint": {
        "x": 96.22386223862239,
        "y": 95.8228782287823,
        "heading": "linear",
        "reverse": false,
        "startDeg": 90,
        "endDeg": 0
      },
      "controlPoints": [],
      "color": "#7BD79C",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmbhml3g-tdo534",
      "name": "LeavePoints",
      "endPoint": {
        "x": 120.12300123001228,
        "y": 71.29397293972941,
        "heading": "linear",
        "reverse": false,
        "startDeg": 0,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#CD975B",
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
      "lineId": "line-071zielfzy93"
    },
    {
      "kind": "path",
      "lineId": "mmbhbpee-uqd8np"
    },
    {
      "kind": "path",
      "lineId": "mmbhc39j-jcmfha"
    },
    {
      "kind": "path",
      "lineId": "mmbhcegu-x06y2x"
    },
    {
      "kind": "path",
      "lineId": "mmbhekkz-21v0fh"
    },
    {
      "kind": "path",
      "lineId": "mmbhf6xa-dlrsnu"
    },
    {
      "kind": "rotate",
      "id": "mmbhhols-ccbiut",
      "name": "Rotate90",
      "degrees": 90,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mmbhhwy8-vxt762"
    },
    {
      "kind": "wait",
      "id": "mmbhm8mp-k4aywv",
      "name": "WaitForRampClear",
      "durationMs": 500,
      "locked": false
    },
    {
      "kind": "path",
      "lineId": "mmbhi8oo-2twafu"
    },
    {
      "kind": "path",
      "lineId": "mmbhml3g-tdo534"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 0,
          "y": 69.5
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 25,
          "y": 144
        },
        {
          "x": 6.5,
          "y": 119
        },
        {
          "x": 6.5,
          "y": 69.5
        }
      ],
      "color": "#dc2626",
      "fillColor": "#fca5a5"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 137.5,
          "y": 119
        },
        {
          "x": 119,
          "y": 144
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 144,
          "y": 69.5
        },
        {
          "x": 137.5,
          "y": 69.5
        }
      ],
      "color": "#0b08d9",
      "fillColor": "#fca5a5"
    }
  ],
  "extraData": {}
}