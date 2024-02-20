#!/usr/bin/python

# Logging tool
# log and print messages
# Author: Bjarne Grossmann
# Copyright 2013 - ZooMoo Ltd

import re
import rclpy.logging as logging

class logMode:
    ALWAYS  = -1
    ERROR   = 0
    WARN    = 1
    OK      = 2
    INFO    = 3
    DEBUG   = 4
    TRACE   = 5

logToLoggingMode = {
    logMode.ERROR: logging.LoggingSeverity.ERROR,
    logMode.WARN: logging.LoggingSeverity.WARN,
    logMode.INFO: logging.LoggingSeverity.INFO,
    logMode.DEBUG: logging.LoggingSeverity.DEBUG,
    logMode.TRACE: logging.LoggingSeverity.DEBUG,
    logMode.ALWAYS: logging.LoggingSeverity.DEBUG,
    logMode.OK: logging.LoggingSeverity.INFO,
}

class logColor:
    UNDERLINE   = '\x1b[4m'
    BOLD        = '\x1b[1m'
    MAGENTA     = '\x1b[35m'
    BLUE        = '\x1b[36m'
    YELLOW      = '\x1b[33m'
    GREEN       = '\x1b[32m'
    RED         = '\x1b[31m'
    ENDC        = '\x1b[0m'

class Log:
    __LOG = []

    __level = logMode.TRACE
    __indent = 0
    __buffered = False
    __useColor = True

    __log = logging.get_logger("skiros")

    # colored messages
    def msgMode(self, mode, msg):
        if (mode == logMode.ALWAYS):
            return msg
        elif (mode == logMode.ERROR):
            return msgError(msg)
        elif (mode == logMode.WARN):
            return msgWarn(msg)
        elif (mode == logMode.INFO):
            return msgInfo(msg)
        elif (mode == logMode.OK):
            return msgOk(msg)
        elif (mode == logMode.DEBUG):
            return msgDebug(msg)
        elif (mode == logMode.TRACE):
            return msgTrace(msg)
        else:
            return msg

    @staticmethod
    def msgTrace(msg):    return                                      msg
    @staticmethod
    def msgDebug(msg):    return logColor.BLUE                      + msg + logColor.ENDC
    @staticmethod
    def msgInfo(msg):     return logColor.BLUE                      + msg + logColor.ENDC
    @staticmethod
    def msgOk(msg):       return logColor.GREEN     + logColor.BOLD + msg + logColor.ENDC
    @staticmethod
    def msgWarn(msg):     return logColor.YELLOW    + logColor.BOLD + msg + logColor.ENDC
    @staticmethod
    def msgError(msg):    return logColor.RED       + logColor.BOLD + msg + logColor.ENDC
    @staticmethod
    def msgHeader(msg):   return logColor.BLUE      + logColor.BOLD + logColor.UNDERLINE + msg + logColor.ENDC
    @staticmethod
    def msgBigHeader(msg):return logColor.BLUE      + logColor.BOLD + "##### " + msg.upper() + " #####" + logColor.ENDC
    @staticmethod
    def msgBoldInfo(msg): return logColor.BLUE      + logColor.BOLD + msg + logColor.ENDC

    # syntactic sugar
    def logAlways(self, msg):              self.log(logMode.ALWAYS, msg)
    def logError(self, msg, desc=None):    self.log(logMode.ERROR, msg, desc)
    def logWarn(self, msg, desc=None):     self.log(logMode.WARN, msg, desc)
    def logOk(self, msg, desc=None):       self.log(logMode.OK, msg, desc)
    def logInfo(self, msg, desc=None):     self.log(logMode.INFO, msg, desc)
    def logDebug(self, msg, desc=None):    self.log(logMode.DEBUG, msg, desc)
    def logTrace(self, msg, desc=None):    self.log(logMode.TRACE, msg, desc)

    def countErrorMsg(self): return countMsg(logMode.ERROR)
    def countWarnMsg(self):  return countMsg(logMode.WARN)
    def countOkMsg(self):    return countMsg(logMode.OK)
    def countInfoMsg(self):  return countMsg(logMode.INFO)
    def countDebugMsg(self): return countMsg(logMode.DEBUG)

    def countTraceMsg(self): return countMsg(logMode.TRACE)

    def hasError(self): return self.hasMode(logMode.ERROR)
    def hasWarn(self):  return self.hasMode(logMode.WARN)
    def hasOk(self):    return self.hasMode(logMode.OK)
    def hasInfo(self):  return self.hasMode(logMode.INFO)
    def hasDebug(self): return self.hasMode(logMode.DEBUG)

    def hasTrace(self): return self.hasMode(logMode.TRACE)

    def assertError(self, cond, msg, desc = None): return self.test(cond, msg, logMode.ERROR, desc)
    def assertWarn(self, cond, msg, desc = None):  return self.test(cond, msg, logMode.WARN, desc)
    def assertOk(self, cond, msg, desc = None):    return self.test(not cond, msg, logMode.OK, desc)
    def assertInfo(self, cond, msg, desc = None):  return self.test(not cond, msg, logMode.INFO, desc)

    def testError(self, cond, msg, descFailed = None, descSuccess = None): return self.test(cond, msg, logMode.ERROR, descFailed, logMode.OK, descSuccess)
    def testWarn(self, cond, msg, descFailed = None, descSuccess = None):  return self.test(cond, msg, logMode.WARN,  descFailed, logMode.OK, descSuccess)

    def lastError(self): return self.lastModeMsg(logMode.ERROR)
    def lastWarn(self):  return self.lastModeMsg(logMode.WARN)

    # set logging level
    def setLevel(self, mode):
        self.__level = mode

    def getLevel(self):
        return self.__level

    # clear log
    def clear(self):
        self.__LOG = []

    # reset log
    def reset(self):
        self.__LOG = []
        self.__indent = 0

    def useColor(self, useColor):
        self.__useColor = useColor

    def enableOutput(self):
        self.__buffered = False

    def disableOutput(self):
        self.__buffered = True

    # set indentation
    def indent(self):
        self.__indent += 2

    def unindent(self):
        self.__indent = max(0, self.__indent - 2)

    # return last message of mode
    def lastModeMsg(self, mode):
        msg = None
        for item in self.__LOG:
            k = item[0]
            v = item[1]
            if k == mode:
                msg = v
        return msg

    # get number of messages
    def countMsg(self, mode):
        count = 0
        for msg in self.__LOG:
            if (msg[0] <= mode) and (msg[0] != logMode.ALWAYS):
                count += 1
        return count

    # add log message
    def log(self, mode, msg, desc=None):
        if (mode <= self.__level):
            msg = msg.rjust(len(msg) + self.__indent)
            self.__LOG.append((mode, msg, desc))
            if not self.__buffered:
                # TODO: HACK! This is a temporary fix to make the logger work with ROS2. Should be "logToLoggingMode[mode]" instead of "logToLoggingMode[logMode.INFO]".
                self.__log.log(self.msgToString(self.__LOG[-1]), logToLoggingMode[logMode.INFO])

    # has log message with mode
    def hasMode(self, mode):
        for msg in self.__LOG:
            if (msg[0] <= mode) and (msg[0] != logMode.ALWAYS):
                return True
        return False

    # test condition and add log message
    def test(self, cond, msg, modeFailed, descFailed=None, modeSuccess=None, descSuccess=None):
        if (not cond):
            self.log(modeFailed, msg, descFailed)
        elif (modeSuccess is not None):
            self.log(modeSuccess, msg, descSuccess)

        return cond

    # convert log to string
    def toString(self):
        resString = ""

        maxLength = 0
        for msg in self.__LOG:
            if (msg[0] == logMode.ALWAYS):
                continue
            m = re.sub("\\x1b\[[0-9]+m", "", msg[1])
            if (len(m) > maxLength):
                maxLength = len(m)

        maxLength += 2
        for msg in self.__LOG:
            k = msg[0]
            m = msg[1]
            d = msg[2]

            if (k == logMode.TRACE):    resString += self.msgTrace(m.ljust(maxLength))
            elif (k == logMode.DEBUG):  resString += self.msgDebug(m.ljust(maxLength))
            elif (k == logMode.INFO):   resString += self.msgInfo(m.ljust(maxLength))
            elif (k == logMode.OK):     resString += m.ljust(maxLength) + self.msgOk("ok")
            elif (k == logMode.WARN):   resString += m.ljust(maxLength) + self.msgWarn("warn")
            elif (k == logMode.ERROR):  resString += m.ljust(maxLength) + self.msgError("error")
            else: resString += m.ljust(maxLength)

            if (d is not None) and (k <= logMode.OK): resString += msgMode(k, ": ")
            if (d is not None): resString += msgMode(k, d)

            resString += "\n"

        if not self.__useColor:
            resString = re.sub("\\x1b\[[0-9]+m", "", resString)

        return resString.strip("\n")

    def msgToString(self, msg):
        k = msg[0]
        m = msg[1]
        d = msg[2]

        maxLength = 20
        resString = ""

        if (k == logMode.TRACE):    resString += self.msgTrace(m.ljust(maxLength))
        elif (k == logMode.DEBUG):  resString += self.msgDebug(m.ljust(maxLength))
        elif (k == logMode.INFO):   resString += self.msgInfo(m.ljust(maxLength))
        elif (k == logMode.OK):     resString += m.ljust(maxLength) + self.msgOk("ok")
        elif (k == logMode.WARN):   resString += m.ljust(maxLength) + self.msgWarn("warn")
        elif (k == logMode.ERROR):  resString += m.ljust(maxLength) + self.msgError("error")
        else: resString += m.ljust(maxLength)

        if (d is not None) and (k <= logMode.OK): resString += msgMode(k, ": ")
        if (d is not None): resString += msgMode(k, d)

        if not self.__useColor: resString = re.sub("\\x1b\[[0-9]+m", "",resString)

        return resString

    def dump(self):
        print(self.toString())

# static functions
_inst = Log()

setLevel    = _inst.setLevel
getLevel    = _inst.getLevel
useColor    = _inst.useColor
enableOutput  = _inst.enableOutput
disableOutput = _inst.disableOutput
clear       = _inst.clear
reset       = _inst.reset
indent      = _inst.indent
unindent    = _inst.unindent

msgMode     = _inst.msgMode
msgTrace    = _inst.msgTrace
msgDebug    = _inst.msgDebug
msgInfo     = _inst.msgInfo
msgOk       = _inst.msgOk
msgWarn     = _inst.msgWarn
msgError    = _inst.msgError
msgHeader   = _inst.msgHeader
msgBigHeader= _inst.msgBigHeader
msgBoldInfo = _inst.msgBoldInfo

lastError   = _inst.lastError
lastWarn    = _inst.lastWarn

countMsg        = _inst.countMsg
countErrorMsg   = _inst.countErrorMsg
countWarnMsg    = _inst.countWarnMsg
countInfoMsg    = _inst.countInfoMsg
countOkMsg      = _inst.countOkMsg

always   = _inst.logAlways
error    = _inst.logError
warn     = _inst.logWarn
ok       = _inst.logOk
info     = _inst.logInfo
debug    = _inst.logDebug
trace    = _inst.logTrace

hasError    = _inst.hasError
hasWarn     = _inst.hasWarn
hasOk       = _inst.hasOk
hasInfo     = _inst.hasInfo
hasDebug    = _inst.hasDebug
hasTrace    = _inst.hasTrace
hasMode     = _inst.hasMode

assertOk    = _inst.assertOk
assertInfo  = _inst.assertInfo
assertError = _inst.assertError
assertWarn  = _inst.assertWarn

testError   = _inst.testError
testWarn    = _inst.testWarn

toString    = _inst.toString
dump        = _inst.dump

ALWAYS  = logMode.ALWAYS
ERROR   = logMode.ERROR
WARN    = logMode.WARN
INFO    = logMode.INFO
OK      = logMode.OK
DEBUG   = logMode.DEBUG
TRACE   = logMode.TRACE


