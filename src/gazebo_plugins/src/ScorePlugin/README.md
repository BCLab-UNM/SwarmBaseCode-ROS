# ScorePlugin README

This plugin implements a Score Plugin for the NASA Swarmathon GUI. There is one required XML tag and a few optional XML tags that can be used to customize the plugin.

| Required XML Tag | Value  | Definition                                                       |
|-----------------:|:------:|:-----------------------------------------------------------------|
|       scoreTopic | string | name of the publishing topic for the score to be used by the GUI |

| Optional XML Tags   | Value               | Definition                                                                           |
|--------------------:|:-------------------:|:-------------------------------------------------------------------------------------|
|collectionZoneRadius | float               | The distance from the center of the nest that a tag must be in to count for scoring. |
|          updateRate | float               | The number of updates per second for the publisher topic.                            |

The following code example demonstrates how to use the plugin in a Collection Disk's SDF configuration file:

```xml
		<!-- Score Plugin -->
		<plugin name="score_sim" filename="libgazebo_plugins_score.so">
			<!-- required: publishing topic for the collection score -->
			<scoreTopic>/collectionZone/score</scoreTopic>

			<!-- optional: the radius used for scoring (default = 0.525) -->
			<collectionZoneRadius>0.525</collectionZoneRadius>

			<!-- optional: updates per second (default = 0.2, i.e. 1 update very 5 seconds) -->
			<updateRate>0.2</updateRate>
		</plugin>
```
