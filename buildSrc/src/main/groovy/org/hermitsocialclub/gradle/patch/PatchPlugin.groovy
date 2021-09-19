package org.hermitsocialclub.gradle.patch

import com.android.build.gradle.BaseExtension
import org.gradle.api.Plugin
import org.gradle.api.Project

/**
 * Dependency patching go brrrr! A Gradle
 * plugin to patch our dependencies so we
 * can hack on them.
 *
 * Gradle doc reference:
 * https://docs.gradle.org/current/userguide/artifact_transforms.html
 *
 * @author Arc'blroth
 */
class PatchPlugin implements Plugin<Project> {
    @Override
    void apply(Project project) {
        def extension = project.extensions.create("hscPatch", PatchExtension)
        project.extensions.findByType(BaseExtension).registerTransform(new PatchTransform(project, extension))
    }
}
