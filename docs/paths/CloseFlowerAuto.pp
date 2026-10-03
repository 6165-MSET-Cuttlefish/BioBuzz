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
      "name": "To Close Flower",
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
      "name": "To Far Flower",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 44.0352380952381,
        "y": 127.13523809523811
      },
      "controlPoints": [
        {
          "x": 30.89111958762887,
          "y": 65.90007319587627
        },
        {
          "x": 24.577319587628867,
          "y": 86.09072164948454
        }
      ],
      "heading": {
        "type": "piecewise",
        "startDeg": 180,
        "endDeg": 65.7979,
        "reverse": false,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.3,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "startDeg": 180,
                "endDeg": 75.2211
              }
            },
            {
              "startProgress": 0.3,
              "endProgress": 1,
              "interpolationType": "tangential",
              "reversed": false,
              "continueFromPrevious": false
            }
          ]
        }
      }
    },
    {
      "id": "line-munigrw7-41wv41",
      "color": "#BAB677",
      "name": "To Hive",
      "locked": false,
      "waitBeforeMs": 0,
      "waitAfterMs": 0,
      "waitBeforeName": "",
      "waitAfterName": "",
      "kind": "atomic",
      "endPoint": {
        "x": 61.37814970197744,
        "y": 71.11449694303039
      },
      "controlPoints": [
        {
          "x": 48.4908885616102,
          "y": 113.20438880706922
        },
        {
          "x": 69.281,
          "y": 106.5252
        }
      ],
      "heading": {
        "type": "piecewise",
        "degrees": 90.5,
        "startDeg": 0,
        "endDeg": 0,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.1,
              "interpolationType": "constant",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "degrees": 64.6358
              }
            },
            {
              "startProgress": 0.1,
              "endProgress": 0.4,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "startDeg": 64.6358,
                "endDeg": -59.2652
              }
            },
            {
              "startProgress": 0.4,
              "endProgress": 1,
              "interpolationType": "tangential",
              "reversed": false,
              "continueFromPrevious": false
            }
          ]
        }
      }
    },
    {
      "id": "line-munin44t-vnde4d",
      "color": "#AABC58",
      "name": "To Audience Wall",
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
          "x": 77.9453,
          "y": 8.5677
        }
      ],
      "heading": {
        "type": "piecewise",
        "startDeg": -102.581,
        "endDeg": -87.5481,
        "reverse": false,
        "piecewiseHeading": {
          "segments": [
            {
              "startProgress": 0,
              "endProgress": 0.2,
              "interpolationType": "linear",
              "reversed": false,
              "continueFromPrevious": false,
              "parameters": {
                "startDeg": -102.581,
                "endDeg": -87.5481
              }
            },
            {
              "startProgress": 0.2,
              "endProgress": 1,
              "interpolationType": "tangential",
              "reversed": false,
              "continueFromPrevious": false
            }
          ]
        }
      }
    },
    {
      "id": "line-muniof4x-vl2dxn",
      "color": "#A5B85A",
      "name": "To Corner",
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
    "xVelocity": 73,
    "yVelocity": 60,
    "aVelocity": 4.64,
    "kFriction": 0.1,
    "rWidth": 15,
    "rHeight": 16,
    "safetyMargin": 1,
    "maxVelocity": 73,
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
  "timestamp": "2026-10-03T22:49:11.054Z"
}
