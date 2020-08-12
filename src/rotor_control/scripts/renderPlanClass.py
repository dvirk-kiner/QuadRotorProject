#!/usr/bin/env python
import re
import os


class createRenderedPlanFile():
    '''
    pathToPlan => Full not relative path to the file we would like to render
    pathToSaveRenderedFile => Full not relative path to the location we want to save tehe rendered file
    dictOfAtions => dictionary of plan actions to tuple containing functions to execute and number of
                    prameters of the plan action, for example:
                    'movebetweensquares' => 'teleport'
                    'movebetweensquares drone location00 location11'
                            ^0            ^1      ^2         ^3

                    'takepic' => (('lowerDrone',-1),3)
                    'takepic drone location12 person1'

    '''

    def __init__(self, pathToPlan, pathToSaveRenderedFile, dictOfAtions, locationDict,toFile = False,mode='teleport'):
        self.toFile = toFile
        self.mode  = mode
        self.locationDict=locationDict
        self.pathToPlan = pathToPlan
        self.pathToSaveRenderedFile = pathToSaveRenderedFile
        self.dictOfAtions = dictOfAtions
        self.listOfPlanAction = [key for key in dictOfAtions]
        # (movebetweensquares [0-9a-zA-Z ?]*)
        self.regexPatterns = [re.compile(
            "("+actionInPlan+" [0-9a-zA-Z ]*)") for actionInPlan in self.listOfPlanAction]
        if os.path.exists(self.pathToSaveRenderedFile):
            os.remove(self.pathToSaveRenderedFile)

    def writeToFile(self, commandToWrite):
        if os.path.exists(self.pathToSaveRenderedFile):
            append_write = 'a'  # append if already exists
        else:
            append_write = 'w'  # make a new file if not
        with open(self.pathToSaveRenderedFile, append_write) as fp:
            fp.write(commandToWrite)

    def convertPlanLineToActionAndSaveIt(self, lineToConvert):
        stringToWrite = ""
        for pattern in self.regexPatterns:
            match = pattern.search(lineToConvert)
            if(match):
                stringToWrite = match.group()
                break
        if stringToWrite != "":
            stringToArr = stringToWrite.split()
            actionsLstToExec = self.dictOfAtions[stringToArr[0]]
            argString=''
            for func,arg in actionsLstToExec:
                if arg>-1:
                    arglocation = self.locationDict[stringToArr[arg]]
                    argString = str(arglocation[0])+","+str(arglocation[1])+","+str(arglocation[2])
                stringToWrite = func + "(" + argString+")\n"
                self.writeToFile(stringToWrite)
            

    def readLinesOfFile(self):
        with open(self.pathToPlan, "r") as fp:
            for _, line in enumerate(fp):
                self.convertPlanLineToActionAndSaveIt(line)





