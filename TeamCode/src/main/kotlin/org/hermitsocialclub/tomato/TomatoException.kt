package org.hermitsocialclub.tomato

/**
 * An exception that is thrown by native code on panic.
 */
class TomatoException(msg: String) : RuntimeException(msg)
