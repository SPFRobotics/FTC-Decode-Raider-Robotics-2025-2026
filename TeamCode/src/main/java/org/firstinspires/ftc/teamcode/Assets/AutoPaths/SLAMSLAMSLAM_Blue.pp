{
  "version": "1.7.5",
  "header": {
    "info": "Created with Pedro Pathing Plus Visualizer",
    "copyright": "Copyright 2026 Matthew Allen. Licensed under the Modified Apache License, Version 2.0.",
    "link": "https://github.com/Mallen220/PedroPathingPlusVisualizer"
  },
  "startPoint": {
    "x": 56.000000000000014,
    "y": 7.999999999999984,
    "heading": "linear",
    "startDeg": 180,
    "endDeg": 90,
    "locked": false
  },
  "lines": [
    {
      "id": "line-h9ylsdtjteu",
      "name": "RunToSpikeOne",
      "endPoint": {
        "x": 48.11627906976744,
        "y": 35.34883720930233,
        "heading": "linear",
        "startDeg": 180,
        "endDeg": 180
      },
      "controlPoints": [
        {
          "x": 51.58139534883721,
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
        "x": 24.79069767441861,
        "y": 35.67441860465115,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
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
        "x": 56.06976744186046,
        "y": 8.651162790697681,
        "heading": "constant",
        "reverse": false,
        "degrees": 180,
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
        "x": 26.760147601476017,
        "y": 11,
        "heading": "constant",
        "degrees": 180
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
        "x": 16.328413284132843,
        "y": 11,
        "heading": "constant",
        "reverse": false,
        "startDeg": 180,
        "endDeg": 90,
        "degrees": 180
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
        "x": 36.39852398523985,
        "y": 11,
        "heading": "constant",
        "degrees": 180
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
        "x": 14.966789667896675,
        "y": 11,
        "heading": "constant",
        "degrees": 180
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
        "x": 56.06976744186046,
        "y": 11,
        "heading": "constant",
        "reverse": false,
        "degrees": 180
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
        "x": 49.19188191881919,
        "y": 26.18450184501845,
        "heading": "linear",
        "reverse": false,
        "degrees": 180,
        "startDeg": 180,
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
          "x": 144,
          "y": 70
        },
        {
          "x": 144,
          "y": 144
        },
        {
          "x": 120,
          "y": 144
        },
        {
          "x": 138,
          "y": 119
        },
        {
          "x": 138,
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
          "x": 6,
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
          "y": 70
        },
        {
          "x": 7,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "extraData": {}
}