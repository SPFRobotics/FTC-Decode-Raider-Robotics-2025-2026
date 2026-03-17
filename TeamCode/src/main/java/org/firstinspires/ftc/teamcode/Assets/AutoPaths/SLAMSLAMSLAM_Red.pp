{
  "version": "1.7.5",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 87.99999999999999,
    "y": 7.999999999999984,
    "heading": "linear",
    "startDeg": 0,
    "endDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-h9ylsdtjteu",
      "name": "RunToSpikeOne",
      "endPoint": {
        "x": 95.88372093023256,
        "y": 35.34883720930233,
        "heading": "linear",
        "startDeg": 0,
        "endDeg": 0
      },
      "controlPoints": [
        {
          "x": 92.41860465116278,
          "y": 25.79069767441861
        }
      ],
      "color": "#788B75",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mm9q3byt-bvphld",
      "name": "IntakeSpikeOne",
      "endPoint": {
        "x": 119.20930232558139,
        "y": 35.67441860465115,
        "heading": "constant",
        "reverse": false,
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#7DA578",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mm9q4avc-aol3pw",
      "name": "ShootSpikeOne",
      "endPoint": {
        "x": 87.93023255813954,
        "y": 8.651162790697681,
        "heading": "constant",
        "reverse": false,
        "degrees": 0,
        "startDeg": 180,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#8A87B6",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmqqtzbz-yf9tgf",
      "endPoint": {
        "x": 117.23985239852398,
        "y": 11,
        "heading": "constant",
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#875888",
      "name": "RunToSlam",
      "eventMarkers": [],
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mm9re6p3-xbq4k6",
      "name": "SlamParking",
      "endPoint": {
        "x": 127.67158671586716,
        "y": 11,
        "heading": "constant",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 90,
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#B96CD7",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmqquvg8-1eekrh",
      "endPoint": {
        "x": 107.60147601476015,
        "y": 11,
        "heading": "constant",
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#896AC5",
      "name": "Reverse",
      "eventMarkers": [],
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmqqv64i-bmgmja",
      "endPoint": {
        "x": 129.03321033210332,
        "y": 11,
        "heading": "constant",
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#CA6B86",
      "name": "ReSlam",
      "eventMarkers": [],
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": ""
    },
    {
      "id": "mmqi7ktz-wnmt2t",
      "name": "ShootParking",
      "endPoint": {
        "x": 87.93023255813954,
        "y": 11,
        "heading": "constant",
        "reverse": false,
        "degrees": 0
      },
      "controlPoints": [],
      "color": "#789758",
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "eventMarkers": []
    },
    {
      "id": "mmqiefm0-3icyb8",
      "name": "Leave",
      "endPoint": {
        "x": 94.80811808118081,
        "y": 26.18450184501845,
        "heading": "linear",
        "reverse": false,
        "degrees": 180,
        "startDeg": 0,
        "endDeg": 90
      },
      "controlPoints": [],
      "color": "#A6AD79",
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
      "lineId": "line-h9ylsdtjteu"
    },
    {
      "kind": "path",
      "lineId": "mm9q3byt-bvphld"
    },
    {
      "kind": "path",
      "lineId": "mm9q4avc-aol3pw"
    },
    {
      "kind": "path",
      "lineId": "mmqqtzbz-yf9tgf"
    },
    {
      "kind": "path",
      "lineId": "mm9re6p3-xbq4k6"
    },
    {
      "kind": "path",
      "lineId": "mmqquvg8-1eekrh"
    },
    {
      "kind": "path",
      "lineId": "mmqqv64i-bmgmja"
    },
    {
      "kind": "path",
      "lineId": "mmqi7ktz-wnmt2t"
    },
    {
      "kind": "path",
      "lineId": "mmqiefm0-3icyb8"
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 0,
          "y": 144
        },
        {
          "x": 24,
          "y": 144
        },
        {
          "x": 6,
          "y": 119
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#dc2626",
      "fillColor": "#ff6b6b"
    },
    {
      "id": "triangle-2",
      "name": "Blue Goal",
      "vertices": [
        {
          "x": 138,
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
          "y": 70
        },
        {
          "x": 137,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "extraData": {}
}