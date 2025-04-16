// pathflip if it sucked in every way but worked with trailblazer
// node scripts/pathflip.mjs -t vertical -i src/main/java/frc/robot/autos/auto_path_commands/red/RedFourPiece2IJKLAuto.java
// node scripts/pathflip.mjs -t color -i src/main/java/frc/robot/autos/auto_path_commands/red/RedStraightLineAuto.java

import { parseArgs } from "node:util";
import assert from "node:assert/strict";
import fs from "node:fs/promises";

const args = parseArgs({
  options: {
    input: { type: "string", short: "i" },
    transform: { type: "string", short: "t", default: "color" },
  },
});

const ALLOWED_TRANSFORMS = ["color", "vertical"];

const { input: inputPath, transform: wantedTransform } = args.values;

assert(
  ALLOWED_TRANSFORMS.includes(wantedTransform),
  "transform should be one of " + ALLOWED_TRANSFORMS.join(", ")
);
assert(inputPath, "input is required");

const inputContents = await fs.readFile(inputPath, "utf8");

const FIELD_LENGTH = 17.55;
const FIELD_HEIGHT = 8.05;
const POSE_2D_REGEXP = /new\s+Pose2d\(([\d\.]*),\s*([\d\.]*)/g;
const ROTATION_2D_REGEXP = /Rotation2d\.fromDegrees\((-?[.\d]+)\)/g;

function stupidRound(value, precision) {
  return Number(value.toFixed(precision));
}

function colorFlip(x, y) {
  const halfFieldLength = FIELD_LENGTH / 2;
  const halfFieldHeight = FIELD_HEIGHT / 2;
  return {
    x: (x - halfFieldLength) * -1 - (x - halfFieldHeight) * 0 + halfFieldLength,
    y: (x - halfFieldLength) * 0 + (y - halfFieldHeight) * -1 + halfFieldHeight,
  };
}

function verticalFlip(x, y) {
  const xAxis = FIELD_HEIGHT / 2;
  return {
    x,
    y: xAxis - (y - xAxis),
  };
}

const TWO_PI = 2 * Math.PI;

/** Normalizes an angle to be within (-pi, pi]. */
function angleModulusRadians(angle) {
  const result = angle - TWO_PI * Math.floor((angle + Math.PI) / TWO_PI);

  if (result === -Math.PI) {
    return Math.PI;
  }

  return result;
}

function angleModulusDegrees(angle) {
  return angleModulusRadians((angle / 180) * Math.PI) * (180 / Math.PI);
}

function transformRotationColor(rotationDeg) {
  return angleModulusDegrees(rotationDeg + 180);
}

function transformRotationVertical(rotationDeg) {
  return angleModulusDegrees(-rotationDeg);
}

function transformColors(value) {
  return multiReplace(value, {
    red: "blue",
    Red: "Blue",
    RED: "BLUE",
    blue: "red",
    Blue: "Red",
    BLUE: "RED",
  });
}

/** @param {string} text */
function transform(text) {
  const firstPass = text
    .replaceAll(POSE_2D_REGEXP, (_, xString, yString) => {
      switch (wantedTransform) {
        case "color": {
          const { x, y } = colorFlip(Number(xString), Number(yString));
          return `new Pose2d(${stupidRound(x, 3)}, ${stupidRound(y, 3)}`;
        }
        case "vertical": {
          const { x, y } = verticalFlip(Number(xString), Number(yString));
          return `new Pose2d(${stupidRound(x, 3)}, ${stupidRound(y, 3)}`;
        }
      }
    })
    .replaceAll(ROTATION_2D_REGEXP, (_, degreesString) => {
      switch (wantedTransform) {
        case "color": {
          return `Rotation2d.fromDegrees(${stupidRound(
            transformRotationColor(Number(degreesString)),
            3
          )})`;
        }
        case "vertical": {
          return `Rotation2d.fromDegrees(${stupidRound(
            transformRotationVertical(Number(degreesString)),
            3
          )})`;
        }
      }
    });

  if (wantedTransform === "color") {
    return transformColors(firstPass);
  }

  return firstPass;
}

if (wantedTransform === "color") {
  await fs.writeFile(transformColors(inputPath), transform(inputContents));
} else {
  await fs.writeFile(inputPath + "_flipped_vertical", transform(inputContents));
}

function multiReplace(string, replacements) {
  const replacementsIterable = Object.entries(replacements);
  let result = "";
  let index = 0;

  while (index < string.length) {
    foundReplace: {
      for (const [searchValue, replaceValue] of replacementsIterable) {
        if (string.slice(index).startsWith(searchValue)) {
          result += replaceValue;
          index += searchValue.length;
          break foundReplace;
        }
      }

      result += string[index++];
    }
  }

  return result;
}
