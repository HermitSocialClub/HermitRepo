package org.hermitsocialclub.gradle.patch

import com.android.build.api.transform.Format
import com.android.build.api.transform.QualifiedContent
import com.android.build.api.transform.Transform
import com.android.build.api.transform.TransformException
import com.android.build.api.transform.TransformInvocation
import org.gradle.api.Project
import org.objectweb.asm.ClassReader
import org.objectweb.asm.ClassWriter
import org.objectweb.asm.tree.ClassNode

import java.util.stream.Collectors

class PatchTransform extends Transform {
    private Project project
    private PatchExtension extension

    PatchTransform(Project project, PatchExtension extension) {
        this.project = project
        this.extension = extension
    }

    @Override
    String getName() {
        return "HSCPatchTransform"
    }

    @Override
    Set<QualifiedContent.ContentType> getInputTypes() {
        return [QualifiedContent.DefaultContentType.CLASSES]
    }

    @Override
    Set<? super QualifiedContent.Scope> getScopes() {
        return [QualifiedContent.Scope.EXTERNAL_LIBRARIES]
    }

    @Override
    boolean isIncremental() {
        return false
    }

    @Override
    void transform(TransformInvocation transformInvocation) throws TransformException, InterruptedException, IOException {
        transformInvocation.outputProvider.deleteAll()

        def classPaths = extension.patchFunctions.keySet().stream().map {
            it.replace('.', '/') + ".class"
        }.collect(Collectors.toSet())

        transformInvocation.inputs.forEach { transformInput ->
            transformInput.jarInputs.forEach { jarInput ->
                def outputJar = transformInvocation.outputProvider.getContentLocation(
                        jarInput.getName(),
                        outputTypes,
                        scopes,
                        Format.JAR
                )
                project.copy {
                    from jarInput.file
                    into outputJar
                }
                project.zipTree(outputJar).matching {
                    it.include(classPaths)
                }.forEach {classFile ->
                    def classReader = null
                    classFile.withInputStream {
                        classReader = new ClassReader(it)
                    }
                    def classNode = new ClassNode()
                    classReader.accept(classNode, 0)

                    def className = classNode.name.replace('/', '.')
                    if (extension.patchFunctions.containsKey(className)) {
                        extension.patchFunctions.get(className).accept(classNode)
                    }

                    def classWriter = new ClassWriter(0)
                    classNode.accept(classWriter)
                    classFile.withOutputStream {
                        it.write(classWriter.toByteArray())
                    }
                }
            }
        }
    }
}
