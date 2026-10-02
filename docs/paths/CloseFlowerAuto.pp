{
  "startPoint": {
    "x": 56,
    "y": 8,
    "locked": false,
    "headingDeg": 90
  },
  "lines": [
    {
      "id": "line-closeflower-1",
      "color": "#ffc516",
      "name": "Close Flower",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 14.9769,
        "y": 46.9989
      },
      "controlPoints": [
        {
          "x": 56,
          "y": 23
        },
        {
          "x": 34.9769,
          "y": 46.9989
        }
      ],
      "heading": {
        "type": "tangential",
        "startDeg": 0,
        "endDeg": 0,
        "reverse": false
      }
    },
    {
      "id": "line-closeflower-2",
      "color": "#84cc16",
      "name": "Path 2 (turn)",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 29.0321,
        "y": 76.6058
      },
      "controlPoints": [
        {
          "x": 23.0138,
          "y": 63.2743
        }
      ],
      "heading": {
        "type": "linear",
        "startDeg": 180,
        "endDeg": 65.7039
      }
    },
    {
      "id": "line-closeflower-3",
      "color": "#22c55e",
      "name": "Path 2",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 46.9748,
        "y": 130.0998
      },
      "controlPoints": [
        {
          "x": 47.087,
          "y": 116.77575831265509
        }
      ],
      "heading": {
        "type": "tangential",
        "startDeg": 0,
        "endDeg": 0,
        "reverse": false
      }
    },
    {
      "id": "line-munigrw7-41wv41",
      "color": "#BAB677",
      "name": "",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 57.001861042183634,
        "y": 101.1650124069479
      },
      "controlPoints": [],
      "heading": {
        "type": "piecewise",
        "reverse": true,
        "degrees": 90.5,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.5,
              "interpolationType": "constant",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "degrees": 90.5
              }
            },
            {
              "startProgress": 0.5,
              "endProgress": 1,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "startDeg": 90.5,
                "endDeg": 270
              }
            }
          ]
        }
      }
    },
    {
      "id": "line-munin44t-vnde4d",
      "color": "#AABC58",
      "name": "",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 45.33250620347395,
        "y": 8.456575682382137
      },
      "controlPoints": [
        {
          "x": 61.57133995037221,
          "y": 0.6836228287841251
        },
        {
          "x": 77.89516129032259,
          "y": 10.267369727047134
        }
      ],
      "heading": {
        "type": "tangential",
        "reverse": false
      }
    },
    {
      "id": "line-muniof4x-vl2dxn",
      "color": "#A5B85A",
      "name": "",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 8.341191066997514,
        "y": 8.330645161290333
      },
      "controlPoints": [],
      "heading": {
        "type": "tangential",
        "reverse": false
      }
    }
  ],
  "shapes": [
    {
      "id": "triangle-1",
      "name": "Red Goal",
      "vertices": [
        {
          "x": 141.5,
          "y": 70
        },
        {
          "x": 141.5,
          "y": 141.5
        },
        {
          "x": 118.3,
          "y": 141.5
        },
        {
          "x": 135.5,
          "y": 118
        },
        {
          "x": 136.3,
          "y": 70.2
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
          "x": 6.2,
          "y": 116.9
        },
        {
          "x": 25,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 141.5
        },
        {
          "x": 0,
          "y": 70
        },
        {
          "x": 6,
          "y": 70
        }
      ],
      "color": "#2563eb",
      "fillColor": "#60a5fa"
    }
  ],
  "sequence": [
    {
      "kind": "path",
      "lineId": "line-closeflower-1"
    },
    {
      "kind": "path",
      "lineId": "line-closeflower-2"
    },
    {
      "kind": "path",
      "lineId": "line-closeflower-3"
    },
    {
      "kind": "path",
      "lineId": "line-munigrw7-41wv41"
    },
    {
      "kind": "path",
      "lineId": "line-munin44t-vnde4d"
    },
    {
      "kind": "path",
      "lineId": "line-muniof4x-vl2dxn"
    }
  ],
  "fieldPoints": [],
  "activePaths": [],
  "settings": {
    "xVelocity": 75,
    "yVelocity": 65,
    "aVelocity": 3.141592653589793,
    "kFriction": 0.1,
    "rWidth": 16,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 40,
    "maxAcceleration": 30,
    "maxDeceleration": 30,
    "fieldMap": "biobuzz.webp",
    "robotImage": "/robot.png",
    "showGhostPaths": false,
    "showOnionLayers": false,
    "onionLayerSpacing": 3,
    "onionColor": "#dc2626",
    "onionNextPointOnly": false,
    "showHeadingArrow": false,
    "showCurrentTValue": false,
    "leftPanelWidth": 0,
    "rightPanelWidth": 496,
    "headingArrowLength": 50,
    "headingArrowColor": "#ffffff",
    "headingArrowThickness": 2,
    "pathOpacity": 1,
    "leftPanelMinWidth": 0,
    "rightPanelMinWidth": 0,
    "penToolMaxPaths": 8,
    "curveThroughMaxPoints": 4,
    "experimentalFeatures": {
      "optimize": false,
      "curveThrough": false
    }
  },
  "version": "1.5.0",
  "timestamp": "2026-10-01T01:14:21.579Z"
}