package org.hermitsocialclub.gradle.patch

import org.objectweb.asm.tree.ClassNode

import java.util.function.Consumer

class PatchExtension {
    protected LinkedHashMap<String, Consumer<ClassNode>> patchFunctions

    /**
     * Adds a ClassNode transformer that will patch the given class.
     * @param className Fully qualified name of the class to patch.
     * @param transformer Transform callback to run on the class ASM tree.
     */
    void patch(String className, Consumer<ClassNode> transformer) {
        this.patchFunctions.put(className, transformer)
    }
}
