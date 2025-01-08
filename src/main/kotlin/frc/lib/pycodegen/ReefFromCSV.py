import csv
with open('Reefs.csv', newline='') as csvfile:
    r = csv.reader(csvfile, delimiter=' ')
    rl = list(r)
    rowIndex = -1

    reefStr = "val reef = arrayOf(\n"


    for row in rl:

        rowIndex += 1
        colIndex = -1
        cDict = {}


        if rowIndex == 6:
            reefStr += ");"
            break

        reefStr += "    FieldLocation(\n"

        for col in row:
            colIndex += 1
            
            match colIndex:
                case 0:
                    cDict["RID"] = col
                    cDict["BID"] = rl[rowIndex+6][colIndex]

                case 1:
                    cDict["RXPOS"] = float(col) / 39.37
                    cDict["BXPOS"] = float(rl[rowIndex+6][colIndex]) / 39.37

                case 2:
                    cDict["RYPOS"] = float(col) / 39.37
                    cDict["BYPOS"] = float(rl[rowIndex+6][colIndex]) / 39.37

                case 3:
                    cDict["RZPOS"] = float(col) / 39.37
                    cDict["BZPOS"] = float(rl[rowIndex+6][colIndex]) / 39.37

                case 4:
                    cDict["RZROT"] = float(col)
                    cDict["BZROT"] = float(rl[rowIndex+6][colIndex])


        reefStr += f'       Pose2d({cDict["RXPOS"]}, {cDict["RYPOS"]}, {cDict["RZROT"]}.rotation2dFromDeg(),\n'
        reefStr += f'       Pose2d({cDict["BXPOS"]}, {cDict["BYPOS"]}, {cDict["BZROT"]}.rotation2dFromDeg()'
        # // TODO: Implement reef positions
        # val reef = arrayOf(
        #     FieldLocation(
        #         Pose2d(0.0, 0.0, 0.0.rotation2dFromDeg()),
        #         Pose2d(0.0, 0.9, 0.0.rotation2dFromDeg())
        #     )
        # )



        reefStr += "\n  ),\n"


print(reefStr)