package us.ilite.logging;

public enum ELevel {
	DEBUG,
	INFO,
	WARN,
	ERROR;


	public static ELevel getForName(String name) {
		for(ELevel aLevel : ELevel.values()) {
			if(aLevel.name().equals(name)) {
				return aLevel;
			}
		}
		return null;
	}
}
