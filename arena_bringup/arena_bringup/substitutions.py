import copy
import dataclasses
import re
import tempfile
import typing

import yaml
import launch
import launch.actions
import launch.substitutions
import launch.conditions
import launch.utilities


def _yaml_iter(obj: list | dict):
    if isinstance(obj, dict):
        return obj.keys()
    elif isinstance(obj, list):
        return range(len(obj))
    else:
        raise RuntimeError()


class LaunchArgument(launch.actions.DeclareLaunchArgument):
    @property
    def substitution(self):
        return launch.substitutions.LaunchConfiguration(self.name)

    @property
    def parameter(self):
        return {self.name: self.substitution}


class SelectAction(launch.Action):
    _actions: dict[str, list[launch.Action]]
    _selector: launch.SomeSubstitutionsType

    def __init__(
        self,
        selector: launch.SomeSubstitutionsType
    ) -> None:
        launch.Action.__init__(self)
        self._actions = {}
        self._selector = selector

    def add(self, value: str, action: launch.Action):
        self._actions.setdefault(value, []).append(
            action
        )

    def execute(self, context: launch.LaunchContext):
        key = launch.utilities.perform_substitutions(
            context, (self._selector,))
        return self._actions.get(key, [])


class YAMLFileSubstitution(launch.Substitution):

    _path: launch.SomeSubstitutionsType
    _default: typing.Optional[dict | typing.Self]
    _substitute: bool

    def __init__(
        self,
        path: launch.SomeSubstitutionsType,
        *,
        default: typing.Optional[dict | typing.Self] = None,
        substitute: bool = False,
    ):
        launch.Substitution.__init__(self)
        self._path = path
        self._default = default
        self._substitute = substitute

    def perform(
        self,
        context: launch.LaunchContext,
    ):
        yaml_path = launch.utilities.perform_substitutions(context, self._path)
        try:
            with open(yaml_path) as f:
                contents = yaml.safe_load(f)
        except BaseException as e:
            if self._default is not None:
                if isinstance(self._default, type(self)):
                    contents = self._default.perform_load(context)
                else:
                    contents = self._default
            else:
                raise e

        if self._substitute:
            def substitute(obj, k):
                if isinstance(obj[k], launch.Substitution):
                    obj[k] = obj[k].perform(context)

            def substitute_recursive(obj: dict | list):
                for k in _yaml_iter(obj):
                    substitute(obj, k)
                    v = obj[k]
                    if isinstance(v, (list, dict)):
                        obj[k] = substitute_recursive(v)

                return obj

            contents = substitute_recursive(contents)

        yaml_file = tempfile.NamedTemporaryFile(mode='w', delete=False)
        yaml.safe_dump(contents, yaml_file)
        yaml_file.close()

        return yaml_file.name

    def perform_load(self, context: launch.LaunchContext) -> dict:
        with open(yaml_path := self.perform(context)) as f:
            content = yaml.safe_load(f)
        if not isinstance(content, dict):
            raise yaml.YAMLError(
                f"{yaml_path} does not contain a top-level dictionary")
        return content

    @classmethod
    def from_dict(cls, obj: dict, /, *, substitute: bool = True):
        return cls(path=[], default=obj, substitute=substitute)


class YAMLMergeSubstitution(launch.Substitution):

    _base: YAMLFileSubstitution
    _yamls: typing.Iterable[YAMLFileSubstitution]

    def __init__(
        self,
        base: YAMLFileSubstitution,
        *yamls: YAMLFileSubstitution,
    ):
        launch.Substitution.__init__(self)
        self._base = base
        self._yamls = yamls

    @classmethod
    def _recursive_merge(cls, obj1: dict, obj2: dict) -> dict:
        for k, v in obj2.items():
            if k in obj1 and isinstance(v, dict):
                v = cls._recursive_merge(obj1[k], v)
            obj1[k] = v
        return obj1

    @classmethod
    def _append_yaml(cls, base: dict, obj: dict) -> dict:
        base = cls._recursive_merge(base, obj)
        return base

    def perform(self, context: launch.LaunchContext):

        combined = copy.deepcopy(self._base.perform_load(context))

        for yaml_path in self._yamls:
            combined = self._append_yaml(
                combined,
                yaml_path.perform_load(context)
            )

        return YAMLFileSubstitution.from_dict(combined).perform(context)


class _YAMLReplacer:
    @dataclasses.dataclass(frozen=True)
    class Replacement:
        ...

    @dataclasses.dataclass(frozen=True)
    class NoReplacement(Replacement):
        value: typing.Any

    @dataclasses.dataclass(frozen=True)
    class StringReplacement(Replacement):
        value: typing.Any

    @dataclasses.dataclass(frozen=True)
    class DictSpreadReplacement(Replacement):
        value: dict

    @dataclasses.dataclass(frozen=True)
    class ListSpreadReplacement(Replacement):
        value: list

    _substitutions: dict

    def _sub_match(self, v: str) -> "Replacement":

        str_v: str | None = v
        replacement: None | _YAMLReplacer.Replacement = None

        while str_v is not None:
            if (match := re.match(r'^\$\{(.*)\}$', str_v)) is None:
                return self.NoReplacement(value=str_v)

            sub, *defaults = match.group(1).split(':-', 1)
            default = defaults[0] if defaults else None

            if sub.startswith('**'):
                if isinstance((substitution := self._substitutions.get(sub[len('**'):])), dict):
                    return self.DictSpreadReplacement(value=substitution)

            if sub.startswith('*'):
                if isinstance((substitution := self._substitutions.get(sub[len('*'):])), list):
                    return self.ListSpreadReplacement(value=substitution)

            if sub in self._substitutions:
                return self.StringReplacement(value=self._substitutions[sub])

            str_v = default

        if replacement is None:
            raise ValueError(f'could not find substitution for {v}')

        return replacement

    def _replace_list(self, obj: list) -> list:
        to_insert: list[tuple[int, list]] = []

        for k, v in enumerate(obj):
            if isinstance(v, str):
                replacement = self._sub_match(v)

                if isinstance(replacement, self.DictSpreadReplacement):
                    raise ValueError('dict spread argument placed outside dict')
                elif isinstance(replacement, self.ListSpreadReplacement):
                    to_insert.append((k, replacement.value))
                    continue
                elif isinstance(replacement, self.StringReplacement):
                    obj[k] = replacement.value

            obj[k] = self.replace(obj[k])

        offset: int = 0
        for i, insertions in to_insert:
            expanded = self._replace_list(insertions)
            obj.pop(i + offset)
            obj[i + offset:i + offset] = expanded
            offset += len(expanded) - 1

        return obj

    def _replace_dict(self, obj: dict) -> dict:
        to_insert: list[tuple[str, dict]] = []

        for k, v in obj.items():

            if isinstance(replacement := self._sub_match(k), self.DictSpreadReplacement):
                to_insert.append((k, replacement.value))
                continue

            if isinstance(v, str):
                replacement = self._sub_match(v)

                if isinstance(replacement, self.DictSpreadReplacement):
                    raise ValueError('dict spreads should be placed in dict keys')
                elif isinstance(replacement, self.ListSpreadReplacement):
                    raise ValueError('list spread argument placed outside list')
                elif isinstance(replacement, self.StringReplacement):
                    obj[k] = replacement.value

            obj[k] = self.replace(v)

        for key, insertions in to_insert:
            obj.pop(key)
            obj.update(self._replace_dict(insertions))

        return obj

    def _replace_str(self, obj: str) -> typing.Any:
        replacement = self._sub_match(obj)
        if isinstance(replacement, self.DictSpreadReplacement):
            raise ValueError('dict spread argument placed outside dict')
        elif isinstance(replacement, self.ListSpreadReplacement):
            raise ValueError('list spread argument placed outside list')
        elif isinstance(replacement, self.StringReplacement):
            return self.replace(replacement.value)
        elif isinstance(replacement, self.NoReplacement):
            return replacement.value
        return obj

    @typing.overload
    def replace(self, obj: dict) -> dict: ...

    @typing.overload
    def replace(self, obj: list) -> list: ...

    @typing.overload
    def replace(self, obj: str) -> str: ...

    def replace(self, obj: typing.Any) -> typing.Any:
        if isinstance(obj, list):
            return self._replace_list(obj)
        if isinstance(obj, dict):
            return self._replace_dict(obj)
        if isinstance(obj, str):
            return self._replace_str(obj)
        return obj

    def __init__(self, substitutions: dict):
        self._substitutions = substitutions


class YAMLReplaceSubstitution(launch.Substitution):

    _substitutions: YAMLFileSubstitution
    _obj: YAMLFileSubstitution

    def __init__(
        self,
        *,
        substitutions: YAMLFileSubstitution,
        obj: YAMLFileSubstitution,
    ):
        launch.Substitution.__init__(self)
        self._substitutions = substitutions
        self._obj = obj

    def perform(self, context: launch.LaunchContext):

        substitutions = self._substitutions.perform_load(context)

        replacer = _YAMLReplacer(substitutions)

        result = replacer.replace(self._obj.perform_load(context))

        return YAMLFileSubstitution.from_dict(
            result,
            substitute=True,
        ).perform(context)
